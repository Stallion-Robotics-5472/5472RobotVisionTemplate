#!/usr/bin/env bash
#
# Offline verification suite for the localization + pathing math.
#
# Compiles the SDK-free parts of TeamCode (geometry, pose estimator, path
# follower, alliance flip) against a tiny stub and runs a set of numeric checks.
# Needs only a JDK -- no Android SDK, no robot, no Gradle.
#
#   ./tools/verify/run.sh
#
# Exits non-zero if any check fails.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$REPO/TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

mkdir -p "$BUILD/src/org/firstinspires/ftc/teamcode"
cp -r "$SRC/lib" "$BUILD/src/org/firstinspires/ftc/teamcode/"
cp -r "$SRC/pathing" "$BUILD/src/org/firstinspires/ftc/teamcode/"
# These two need the real Android/FTC SDK, so they are out of scope here.
rm -f "$BUILD/src/org/firstinspires/ftc/teamcode/pathing/MecanumDrivetrain.java" \
      "$BUILD/src/org/firstinspires/ftc/teamcode/pathing/PathPlannerServer.java" \
      "$BUILD/src/org/firstinspires/ftc/teamcode/pathing/"*.md
cp -r "$REPO/tools/verify/stub/"* "$BUILD/src/"
cp -r "$REPO/tools/verify/src/qa" "$BUILD/src/"

javac -nowarn -d "$BUILD/out" $(find "$BUILD/src" -name '*.java')

# The gamepad turn polarity lives in the OpModes, which need the SDK to compile.
# Read it straight out of the source so the check tracks the real code.
if grep -q 'double turn = -gamepad1.right_stick_x;' "$SRC/opmodes/RobotCentricDrive.java"; then
  POLARITY=-1.0
else
  POLARITY=1.0
fi

STATUS=0
for T in Geom3dTest DriveTest EstimatorTest CodegenCompileTest; do
  echo "=============== $T ==============="
  if ! java -Drepo.root="$REPO" -cp "$BUILD/out" "qa.$T" "$POLARITY"; then STATUS=1; fi
  echo
done
echo "=============== FollowerSimTest ==============="
java -Drepo.root="$REPO" -cp "$BUILD/out" qa.FollowerSimTest
echo
if [ "$STATUS" -ne 0 ]; then
  echo "SOME CHECKS FAILED"
else
  echo "All checks passed."
fi
exit "$STATUS"
