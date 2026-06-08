"""USB serial command console for the fixed SwivelCut arm."""

from swivelcut import SwivelCut


HELP = """Commands:
  ARM FOLDED          confirm the physical folded pose and enable the drivers
  DISARM              disable both motor drivers
  J1 <deg>            move shoulder to an absolute angle
  J2 <deg>            move elbow to an absolute angle
  ANGLES <j1> <j2>    move both joints to absolute angles
  XY <x> <y> [UP|DOWN]
                      move the blade to an absolute position in millimetres
  XYJ1 <x> <y> [UP|DOWN]
                      solve XY and move only J1
  XYJ2 <x> <y> [UP|DOWN]
                      solve XY and move only J2
  CUT <x0> <y0> <x1> <y1> [UP|DOWN]
                      cut a straight Cartesian line
  POS                 print x, y, J1 and J2
  HELP                show this list

Angles are degrees. Positive angles are counterclockwise viewed from above.
"""


class SwivelCutConsole:
    def __init__(self, arm=None, output=print):
        self.arm = arm if arm is not None else SwivelCut()
        self.output = output
        self.armed = False

    def _require_armed(self):
        if not self.armed:
            raise ValueError("arm is disabled; place it folded and type ARM FOLDED")

    @staticmethod
    def _elbow(value):
        elbow = value.lower()
        if elbow not in ("up", "down"):
            raise ValueError("elbow must be UP or DOWN")
        return elbow

    def execute(self, line):
        parts = line.strip().split()
        if not parts:
            return
        command = parts[0].upper()

        if command == "HELP":
            self.output(HELP)
        elif command == "ARM":
            if len(parts) != 2 or parts[1].upper() != "FOLDED":
                raise ValueError("use ARM FOLDED after physically folding the arm")
            self.arm.disable()
            self.arm.set_folded_start()
            self.arm.enable()
            self.armed = True
            self.output("ARMED: folded pose set to J1=0.0, J2=180.0")
        elif command == "DISARM":
            self.arm.disable()
            self.armed = False
            self.output("DISARMED")
        elif command == "J1":
            self._require_armed()
            self._expect_count(parts, 2, "J1 <deg>")
            self.arm.move_joint(1, float(parts[1]))
            self._print_position()
        elif command == "J2":
            self._require_armed()
            self._expect_count(parts, 2, "J2 <deg>")
            self.arm.move_joint(2, float(parts[1]))
            self._print_position()
        elif command == "ANGLES":
            self._require_armed()
            self._expect_count(parts, 3, "ANGLES <j1> <j2>")
            self.arm.move_to_angles(float(parts[1]), float(parts[2]))
            self._print_position()
        elif command == "XY":
            self._require_armed()
            x, y, elbow = self._xy_args(parts, "XY")
            self.arm.move_to_xy(x, y, elbow)
            self._print_position()
        elif command == "XYJ1":
            self._require_armed()
            x, y, elbow = self._xy_args(parts, "XYJ1")
            self.arm.move_joint_to_xy(1, x, y, elbow)
            self._print_position()
        elif command == "XYJ2":
            self._require_armed()
            x, y, elbow = self._xy_args(parts, "XYJ2")
            self.arm.move_joint_to_xy(2, x, y, elbow)
            self._print_position()
        elif command == "CUT":
            self._require_armed()
            if len(parts) not in (5, 6):
                raise ValueError("use CUT <x0> <y0> <x1> <y1> [UP|DOWN]")
            elbow = self._elbow(parts[5]) if len(parts) == 6 else "up"
            coordinates = [float(value) for value in parts[1:5]]
            self.arm.cut_line(*coordinates, elbow=elbow)
            self._print_position()
        elif command == "POS":
            self._print_position()
        else:
            raise ValueError("unknown command; type HELP")

    @staticmethod
    def _expect_count(parts, count, usage):
        if len(parts) != count:
            raise ValueError("use " + usage)

    def _xy_args(self, parts, command):
        if len(parts) not in (3, 4):
            raise ValueError("use {} <x> <y> [UP|DOWN]".format(command))
        elbow = self._elbow(parts[3]) if len(parts) == 4 else "up"
        return float(parts[1]), float(parts[2]), elbow

    def _print_position(self):
        x, y, t1, t2 = self.arm.position()
        self.output(
            "POS x={:.1f} y={:.1f} J1={:.2f} J2={:.2f}".format(x, y, t1, t2)
        )

    def shutdown(self):
        self.arm.disable()
        self.armed = False


def run_console():
    console = SwivelCutConsole()
    print("SwivelCut USB console")
    print("Physically fold the arm, then type: ARM FOLDED")
    print("Type HELP for all commands.")
    try:
        while True:
            try:
                console.execute(input("swivelcut> "))
            except (ValueError, TypeError) as error:
                print("ERROR:", error)
    except (KeyboardInterrupt, EOFError):
        print("\nEmergency stop: drivers disabled")
    finally:
        console.shutdown()
