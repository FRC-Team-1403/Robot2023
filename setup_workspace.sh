#!/bin/bash

# Customize this repository template with the custom package names for
# the intended robot.
#
# Usage: setup_workspace.sh <Robot Name>
#
# The <Robot Name> should be the human readable name. This will get
# converted to a proper java package name by converint it to lower case
# and stripping out the spaces.
#
# Keep the name short and simple, but unique for this robot so that
# the names will not clash if it lives in the same codebase as another robot.
# Hint -- you can end the name with the year (or YY) if that makes sense.


# Fail on error
set -e

if [[ $# != 1 ]]; then
  >&2 echo "Usage: $0 <robot name>"
  exit -1
fi

# The human-readable name
ROBOT_NAME="$1"

# Convert robot name to lower case, stripping out spaces.
# This will be the java package name for it.
PACKAGE_NAME=$(echo "${ROBOT_NAME}" | tr '[:upper:]' '[:lower:]' | tr -d ' ')

if [[ ! "$PACKAGE_NAME" =~ ^[a-z][a-z0-9_]+$ ]]; then
  >&2 echo "ERROR: Invalid robot name '${ROBOT_NAME}'"
  >&2 echo "       Robot names must be alphanumberic and start with a letter."
  exit -1
fi



# Ensure that we are in a clean workspace before running this script.
# Some operations we perform (e.g. fixing robot name) will commit changes
# into the repository. If it is clean then we know we are not unintentionally
# commiting other things.
function check_workspace_or_die() {
  changes=$(git diff --stat)
  if [[ "$changes" == "" ]]; then
    echo "Workspace looks clean"
    return
  fi
  >&2 echo "You cannot run this script in a dirty workspace:\n$changes"
  exit -1
}

# Change occurances of "__replaceme__" with this robot's name.
# This includes the following locations:
#    * README.md
#    * Robot/build.gradle
#    * Robot/src/java/main/team1403/__replaceme__
#
# The skeleton classes will also be moved to the appropriate location
# for the infered java package name.
function fix_robot_name() {
  if [[ ! -d Robot/src/main/java/team1403/robot/__replaceme__ ]]; then
    echo "This robot is already named. It is probably '$(head README.md)'"
    return
  fi

  # Replace the name in the README.md with the human-readable name
  sed -i "s/__replaceme__/${ROBOT_NAME}/g" README.md

  # Replace our custom package reference to the 'Main' class in build.gradle.
  sed -i "s/\.__replaceme__/\.${PACKAGE_NAME}/g" Robot/build.gradle  

  # Replace our custom package name in the Java code template.
  find Robot/src -type f \
                 -exec sed -i "s/\.__replaceme__/\.${PACKAGE_NAME}/g" {} \;

  # Place the package in the right path
  git mv Robot/src/main/java/team1403/robot/__replaceme__ \
         Robot/src/main/java/team1403/robot/${PACKAGE_NAME}

  echo "Created '${ROBOT_NAME}' in package team1403.robot.${PACAKGE_NAME}"
}


# The .gitignore says to ignore .classpath files
# I am not sure why, so we'll stick to that policy.
#
# However vscode cannot find the tests in our multiproject.
# We need to give it a hint as to where the tests are.
# So we'll write .classpath locally (.gitignore prevents it
# from being submitted)
function fix_classpath() {
  if [[ -f .classpath ]]; then
    echo << EOF
WARNING: There is already a .classpath.

Make sure it has the src/test/java directories in it.
If these are missing then it may have trouble finding the tests to run.
EOF
    return
  fi
  cat <<EOF > .classpath
<?xml version="1.0" encoding="UTF-8"?>
<classpath>
    <classpathentry kind="src" output="target/classes"
                    path="src/test/java">
        <attributes>
          <attribute name="optional" value="true"/>
          <attribute name="test" value="true"/>
        </attributes>
    </classpathentry>
</classpath>
EOF

  echo "Created a .classpath file to help vscode."
}

check_workspace_or_die

git checkout -b initial_setup
fix_robot_name
git commit -a -m "Customized template our robot '$ROBOT_NAME'"

fix_classpath
if [[ $(git diff --stat) ]]; then
  >&2 echo "ERROR: Unexpected changes -- this script is broken."
  exit -1
fi

echo "Testing..."
if ! ./gradlew test; then
    >&2 "Something is broken. Failed"
    exit -1
fi

if [[ $(git diff --stat) ]]; then
  >&2 echo "ERROR: Unexpected changed sources after testing."
  exit -1
fi

echo "Pushing initial repository to $(git get-url origin)..."
git push origin initial_setup

echo <<EOF

To complete the setup, go to your GitHub repository and review
the 'initial_setup' that was just pushed. If it looks good, then
merge the changes into the 'main' branch. You can delete the branch
in github with the merge.

Then come back to this workspace and resync it to your origin.

git checkout main
git pull origin main
git diff main initial_setup  # confirm no changes
git branch -D initial_setup  # no longer need our local branch


You are now ready to start customizing your robot in the
$PACKAGE_NAME Java package.

EOF
