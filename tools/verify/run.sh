#!/usr/bin/env bash
#
# Offline verification suite.
#
# Phase 1 typechecks the whole TeamCode tree against a small FTC SDK stub, so a
# compile error shows up without Android Studio. Phase 2 runs numeric checks on
# the localization and pathing math, including the Localization subsystem itself
# driven against synthetic Limelight frames.
#
# Needs only a JDK -- no Android SDK, no robot, no Gradle.
#
#   ./tools/verify/run.sh
#
# Exits non-zero if anything fails.
#
# NOTE: the stubs under tools/verify/stub are a hand-written subset of the FTC
# SDK, present so this can run offline. A green run means the logic is sound and
# the code typechecks against that subset -- it is not a substitute for building
# the real app before a competition.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEAMCODE="$REPO/TeamCode/src/main/java"
SRC="$TEAMCODE/org/firstinspires/ftc/teamcode"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

STATUS=0

# --------------------------------------------------------------------------
# Phase 1: typecheck everything against the SDK stubs.
# --------------------------------------------------------------------------
echo "=============== Typecheck (TeamCode vs SDK stubs) ==============="
# PathPlannerServer is excluded: it binds to Android and NanoHTTPD internals
# that are not worth stubbing. It is verified on hardware instead.
TEAM_FILES=$(find "$SRC" -name '*.java' ! -name 'PathPlannerServer.java')
STUB_FILES=$(find "$REPO/tools/verify/stub" -name '*.java')
if javac -nowarn -d "$BUILD/app" $TEAM_FILES $STUB_FILES 2>&1; then
  echo "$(echo "$TEAM_FILES" | wc -l | tr -d ' ') team files compile cleanly."
else
  echo "TYPECHECK FAILED"
  STATUS=1
fi
echo

# --------------------------------------------------------------------------
# Phase 2: numeric checks.
# --------------------------------------------------------------------------
javac -nowarn -d "$BUILD/out" -cp "$BUILD/app" \
  $(find "$REPO/tools/verify/src/qa" -name '*.java') $TEAM_FILES $STUB_FILES

# The gamepad turn polarity lives in an OpMode, so read it from the source and
# pass it in rather than keeping a second copy of it in the test.
if grep -q 'double turn = -gamepad1.right_stick_x;' "$SRC/opmodes/RobotCentricDrive.java"; then
  POLARITY=-1.0
else
  POLARITY=1.0
fi

for T in Geom3dTest DriveTest EstimatorTest HeadingTrustTest CommandTest \
         ShooterMapTest AimLogicTest GoalSelectorTest CodegenCompileTest; do
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
