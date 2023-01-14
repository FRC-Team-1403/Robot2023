Creating new Robots
---

**DO NOT create team-related repositories in personal github accounts.**.

These robots contain private code. You may clone public repositories into your own account.

---

1. Make sure you completed [Setup Workspace](setup_workspace.md).

1. Navigate to the [TemplateRobotRepository](https://github.com/FRC-Team-1403/PublicCougarRobotTemplate) github page

1. Click the green "Use this template" button.

1. Give your repository an approrpiate name.

1. Clone it into a local workspace on your machine.
   Below is pretending your new repository is called "MyRobot"
   And your team repository is on github as "FRC-Team-1403".

   ```
   BASE_REPO_URL=https://github.com/FRC-Team-1403
   NEW_REPO_NAME=MyRobot
   git clone ${{BASE_REPO_URL}/${NEW_REPO_NAME}.git
   cd $NEW_REPO_NAME
   ```

1. Run the setup script in the directory.

   This script will customize your workspace and push the baseline
   back to GitHub.
   ```
   NEW_ROBOT_NAME="My Robot"
   ./setup_repository.sh "$NEW_ROBOT_NAME"
   ```

1. Go back to GitHub and create a PR for the 'initial_setup' branch.
   * Review the changes and accept them back into the main branch.

1. Resync your local workspace and remove the initial_setup branch.
   ```
   git checkout main
   git pull origin main
   git diff initial_setup
   git branch -D initial_setup
   ```
