"""
Cell: a model of one leaf standard cell built straight from its GDS layout
-- its transistors, resolved nets, and named input/output pins (reusing
pin.LeafCellAnalyzer for the layout-derived connectivity and
pin-direction classification) -- wired into two different circuit
representations of the same transistor network:

  - a PySpice circuit (simulate()), an analog DC operating-point
    simulation with real device models, answering "what voltage comes out
    for these input voltages".
  - a z3 switch-level model (simulate_z3()/find_inputs_for_output()),
    ideal boolean switches with no device physics, answering purely
    logical questions -- "what does this cell compute" and "which inputs
    make the output 1" -- via SAT solving rather than simulation.

Both share the same net resolution (`Cell._net_name()`), so they're two
independent checks against the same extracted transistor list -- if they
disagree, the layout extraction (or one of the two models) has a bug.

Net naming: a transistor's raw gate/source/drain labels (e.g. "GN_9",
"DN_2") are internal layout labels, not stable circuit net names -- two
transistors sharing a diffusion region only land on the same SPICE net if
that label is first resolved through NetTracer's electrical-equivalence
graph. `Cell._net_name()` does that resolution once per terminal and
settles on one of three forms: "in_<pin>" / "out_<pin>" for a terminal on
one of the cell's own named pins, the literal SPICE ground node for
anything tied to VSS, or the raw internal label for a purely internal node
(kept only so shared internal nets still land on the same SPICE net as
each other).

Device models: NMOS_PARAMS/PMOS_PARAMS below are generic level-1 SPICE
parameters, not calibrated to sky130's real BSIM4 models -- this project
doesn't have access to that PDK's SPICE deck, only its GDS layout. They're
good enough to get correct pull-up/pull-down logic behavior (a PMOS turns
on near VGS=0, an NMOS turns on near VGS=VDD) for a functional check of
what a cell computes, not a timing- or power-accurate simulation.
"""

import z3
from PySpice.Spice.Netlist import Circuit

from pin import LeafCellAnalyzer
from transistor import TransistorType, transistor_to_pyspice, transistor_to_z3

PIN_TEXTTYPE = 5
POWER_NAMES = {"VPWR", "VDD", "VPB", "VGND", "VSS", "GND", "VNB"}
GROUND = 0  # PySpice's node for SPICE's ground/reference node "0"

NMOS_MODEL = "NMOS_MODEL"
PMOS_MODEL = "PMOS_MODEL"
NMOS_PARAMS = dict(level=1, vto=0.4, kp=200e-6, lambda_=0.02)
PMOS_PARAMS = dict(level=1, vto=-0.4, kp=100e-6, lambda_=0.02)


def _pin_net(prefix, pin_name):
    return f"{prefix}_{pin_name}"


class Cell:
    """One leaf standard cell, electrically modeled from its GDS layout.

    Args:
        gds_file: path to the .gds file.
        cell_name: name of the leaf cell inside it (e.g.
            "sky130_fd_sc_hd__and3_2").
        vdd: supply voltage in volts -- used for the VDD rail and as the
            logic-1 drive level for inputs during simulate().
        logic_threshold: fraction of `vdd` above which an output node's
            simulated voltage reads back as logic 1 (default 0.5 = VDD/2).

    Attributes:
        transistors: list of transistor.Transistor for this cell.
        input_labels, output_labels: this cell's own named pins (per
            LeafCellAnalyzer.classify_pin_direction), power/ground
            excluded, sorted for a stable simulate() argument order.
        circuit: the static PySpice Circuit built from `transistors` --
            just the MOSFETs and their models, for inspection (e.g.
            printing the netlist). simulate() builds its own fresh copy
            with power/input sources added rather than mutating this one
            (PySpice's Circuit.clone() can't round-trip a circuit that has
            .model() cards in this PySpice version -- DeviceModel.clone()
            passes its parameters positionally into an __init__ that only
            accepts them as **kwargs -- so simulate() calls
            `self._build_circuit()` again instead of cloning).
    """

    def __init__(self, gds_file, cell_name, vdd=1.8, logic_threshold=0.5):
        self.gds_file = gds_file
        self.cell_name = cell_name
        self.vdd = vdd
        self.logic_threshold = logic_threshold

        self._analyzer = LeafCellAnalyzer.for_cell(gds_file, cell_name)
        self.transistors = self._analyzer.transistors

        pin_names = sorted({
            lbl.text for lbl in self._analyzer.labels
            if lbl.texttype == PIN_TEXTTYPE and lbl.text not in POWER_NAMES
        })
        self.input_labels = [n for n in pin_names if self._analyzer.classify_pin_direction(n) == "input"]
        self.output_labels = [n for n in pin_names if self._analyzer.classify_pin_direction(n) == "output"]

        self.circuit = self._build_circuit()
        self.z3_circuit, self.z3_inputs, self.z3_outputs = self.build_z3_circuit()

    def _net_name(self, label):
        """Resolve a transistor terminal's raw label to a stable net name:
        "in_<pin>"/"out_<pin>" if it's one of this cell's own named pins,
        the simulator's ground node if it's tied to VSS, "VDD" if tied to
        VDD, or else the union-find's own canonical key for a purely
        internal node.

        Deliberately uses the raw union-find root (`tracer._uf.find`), not
        `tracer.find_root()` -- find_root() picks "the alphabetically
        first *other* member of the equivalence class", which depends on
        which label you started from and so gives DIFFERENT answers for
        two labels on the very same net (e.g. find_root("DN_0") and
        find_root("GP_0") can differ even when DN_0 and GP_0 are
        electrically the same node). That's fine for pipeline.py's
        one-row-at-a-time display, but not for building a netlist, where
        every terminal on the same net must resolve to the identical
        SPICE node name. `_uf.find()` is a true union-find root and is
        consistent no matter which member of the set you query.
        """
        uf = self._analyzer.tracer._uf
        net_key = uf.find(label)

        if net_key == uf.find("VSS"):
            return GROUND
        if net_key == uf.find("VDD"):
            return "VDD"

        pin_name, direction = self._analyzer.pin_name_for_net(label)
        if pin_name is None:
            return net_key  # purely internal node -- any consistent name will do

        prefix = {"input": "in", "output": "out"}.get(direction, direction)
        return _pin_net(prefix, pin_name)

    def _build_circuit(self):
        circuit = Circuit(self.cell_name)
        circuit.model(NMOS_MODEL, "nmos", **NMOS_PARAMS)
        circuit.model(PMOS_MODEL, "pmos", **PMOS_PARAMS)

        for t in self.transistors:
            if t.kind not in (TransistorType.NMOS, TransistorType.PMOS) or None in (t.source_label, t.drain_label):
                print(f"warning: skipping transistor {t.gate_label} with incomplete terminals "
                      f"(kind={t.kind}, source={t.source_label}, drain={t.drain_label})")
                continue

            bulk_net = GROUND if t.kind == TransistorType.NMOS else "VDD"
            transistor_to_pyspice(
                circuit, t,
                gate_net=self._net_name(t.gate_label),
                source_net=self._net_name(t.source_label),
                drain_net=self._net_name(t.drain_label),
                bulk_net=bulk_net,
                nmos_model=NMOS_MODEL, pmos_model=PMOS_MODEL,
            )
        return circuit

    def _z3_net_var(self, net_vars, label):
        """Resolve a transistor terminal's raw label to a z3 BoolRef,
        sharing one variable per electrical net (same net resolution as
        `_net_name()`, cached in the `net_vars` dict the caller passes
        in) -- VDD/VSS become concrete z3.BoolVal constants rather than
        variables, since they're not free to solve for."""
        name = self._net_name(label)
        if name == GROUND:
            return z3.BoolVal(False)
        if name == "VDD":
            return z3.BoolVal(True)
        return net_vars.setdefault(name, z3.Bool(str(name)))

    def build_z3_circuit(self):
        """Build a z3 switch-level model of this cell's transistor network.

        Each transistor becomes one z3 constraint (transistor.transistor_to_z3):
        an ideal switch that shorts its source and drain together while
        its gate says it should conduct, and asserts nothing otherwise.
        Combined, these constraints let z3 propagate connectivity from
        VDD/VSS through however many transistors are actually conducting
        for a given input assignment -- there's no separate graph-
        reachability step; it falls out of z3 solving the equalities.

        Returns:
            (circuit, input_vars, output_vars) where `circuit` is a single
            z3 BoolRef (the AND of every transistor's constraint) and
            `input_vars`/`output_vars` are {pin_name: z3.BoolRef} for
            `self.input_labels`/`self.output_labels`.
        """
        net_vars = {}
        constraints = []
        for t in self.transistors:
            if t.kind not in (TransistorType.NMOS, TransistorType.PMOS) or None in (t.source_label, t.drain_label):
                continue
            constraints.append(transistor_to_z3(
                t,
                self._z3_net_var(net_vars, t.gate_label),
                self._z3_net_var(net_vars, t.source_label),
                self._z3_net_var(net_vars, t.drain_label),
            ))

        input_vars = {name: net_vars.setdefault(_pin_net("in", name), z3.Bool(_pin_net("in", name)))
                      for name in self.input_labels}
        output_vars = {name: net_vars.setdefault(_pin_net("out", name), z3.Bool(_pin_net("out", name)))
                       for name in self.output_labels}
        return z3.And(*constraints), input_vars, output_vars

    def simulate(self, inputs):
        """Run a DC operating-point simulation and read back this cell's
        output pin(s).

        Args:
            inputs: logic values (0/1, or anything truthy/falsy) for
                `self.input_labels`, positionally in that order. Each
                value drives that input's net to `self.vdd` (logic 1) or
                0V (logic 0) for the simulation.

        Returns:
            A single 0/1 int if this cell has exactly one output pin,
            else a list of 0/1 ints in `self.output_labels` order.
        """
        inputs = list(inputs)
        if len(inputs) != len(self.input_labels):
            raise ValueError(
                f"expected {len(self.input_labels)} input value(s) for "
                f"{self.input_labels}, got {len(inputs)}"
            )

        circuit = self._build_circuit()
        circuit.V("dd_supply", "VDD", GROUND, self.vdd)
        for i, (label, value) in enumerate(zip(self.input_labels, inputs)):
            level = self.vdd if value else 0.0
            circuit.V(f"in_{i}", _pin_net("in", label), GROUND, level)

        simulator = circuit.simulator(temperature=25, nominal_temperature=25)
        analysis = simulator.operating_point()

        outputs = [
            # operating-point analysis gives one sample per node -- index
            # [0] rather than float() the node directly, since numpy no
            # longer implicitly converts a shape-(1,) array to a scalar.
            int(float(analysis[_pin_net("out", label)][0]) >= self.vdd * self.logic_threshold)
            for label in self.output_labels
        ]
        return outputs

    def simulate_z3(self, inputs):
        """Same contract as simulate(), but evaluated with z3's switch-
        level model instead of an ngspice simulation -- a fast, purely
        logical cross-check of what simulate() measures electrically.

        Args:
            inputs: logic values (0/1, or anything truthy/falsy) for
                `self.input_labels`, positionally in that order.

        Returns:
            A list of 0/1 ints in `self.output_labels` order.
        """
        inputs = list(inputs)
        if len(inputs) != len(self.input_labels):
            raise ValueError(
                f"expected {len(self.input_labels)} input value(s) for "
                f"{self.input_labels}, got {len(inputs)}"
            )

        solver = z3.Solver()
        solver.add(self.z3_circuit)
        for name, value in zip(self.input_labels, inputs):
            solver.add(self.z3_inputs[name] == bool(value))

        given = dict(zip(self.input_labels, inputs))
        outputs = []
        for name in self.output_labels:
            var = self.z3_outputs[name]
            # A well-formed complementary CMOS gate has exactly one of its
            # pull-up/pull-down networks conducting for any input, which
            # pins every net (including this output) to a single value --
            # so don't just trust whatever model solver.check() happens
            # to hand back; confirm the *other* value is unreachable too.
            solver.push()
            solver.add(var == True)
            can_be_high = solver.check() == z3.sat
            solver.pop()
            solver.push()
            solver.add(var == False)
            can_be_low = solver.check() == z3.sat
            solver.pop()

            if can_be_high and not can_be_low:
                outputs.append(1)
            elif can_be_low and not can_be_high:
                outputs.append(0)
            elif can_be_high and can_be_low:
                raise ValueError(
                    f"output {name!r} isn't uniquely determined by inputs {given} -- "
                    f"the extracted switch network leaves it floating for this input, "
                    f"which shouldn't happen for a complementary CMOS gate"
                )
            else:
                raise ValueError(f"inputs {given} make this cell's switch network unsatisfiable")
        return outputs

    def find_inputs_for_output(self, target=1, output_label=None):
        """Find every input combination that makes one output pin equal
        `target`, via z3 SAT enumeration rather than brute-force
        simulation of all 2**len(input_labels) combinations.

        Args:
            target: the output value (0/1) to solve for.
            output_label: which entry of `self.output_labels` to solve
                for. Required if this cell has more than one output;
                defaults to the only one otherwise.

        Returns:
            A list of tuples, each a full assignment for `self.input_labels`
            (in that order) for which `output_label` == `target`.
        """
        if output_label is None:
            if len(self.output_labels) != 1:
                raise ValueError(
                    f"cell has {len(self.output_labels)} outputs {self.output_labels}; "
                    f"pass output_label explicitly"
                )
            output_label = self.output_labels[0]

        solver = z3.Solver()
        solver.add(self.z3_circuit)
        solver.add(self.z3_outputs[output_label] == bool(target))

        input_vars = [self.z3_inputs[name] for name in self.input_labels]
        solutions = []
        while solver.check() == z3.sat:
            model = solver.model()
            values = tuple(
                int(z3.is_true(model.eval(v, model_completion=True)))
                for v in input_vars
            )
            solutions.append(values)
            # block this exact input combination so the next check() is
            # forced to either find a different one or come back unsat
            solver.add(z3.Or([v != z3.BoolVal(bool(val)) for v, val in zip(input_vars, values)]))
        return solutions


if __name__ == "__main__":
    import sys

    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    cell_name = sys.argv[2] if len(sys.argv) > 2 else "sky130_fd_sc_hd__and3_2"

    cell = Cell(gds_file, cell_name)
    print(f"{cell_name}: inputs={cell.input_labels} outputs={cell.output_labels}")

    from itertools import product
    print("PySpice simulation:")
    for combo in product((0, 1), repeat=len(cell.input_labels)):
        print(f"  {dict(zip(cell.input_labels, combo))} -> {cell.simulate(combo)}")

    print("Z3 simulation:")
    for combo in product((0, 1), repeat=len(cell.input_labels)):
        print(f"  {dict(zip(cell.input_labels, combo))} -> {cell.simulate_z3(combo)}")

    print(f"Search for inputs that satisfy the gate: -> {cell.z3_inputs}:{cell.find_inputs_for_output()}")
