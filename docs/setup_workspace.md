Setup Workspace
===


# Install WPILib

Follow the official
[WPILib Installation Guide]](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
if you have not yet setup your machine for the WPI libraries.



# Team-1403 Additional Setup

## Platform-Specific Setup
### Windows

1. Install one of the following bash shells:
    * [git bash](https://gitforwindows.org/)
      Your Windows home directory will be in /c/users/<username>.
      I know this works with our use case because it is what I've been using.
      
    * [Windows Subsystem for Linux (wsl)](https://docs.microsoft.com/en-us/windows/wsl/install)
      Your Windows home directory will be in /mnt/c/users/<username>.
      Eventually this will be the recommendation but I am being cautious because I have not yet
      used it with this toolchain so do not know if there are going to be issues to work out.

      If you choose WSL, see [Setting up Windows Subsystem For Linux](setting_up_wsl.md)
      for more instructions specific to our team.
 
### MacOS

No platform-specific setup needed.

### Linux

No platform-specific setup needed.

----

## Visual Code Setup
1. In Visual Code, install the following from the marketplace :
   * [GitLens](https://marketplace.visualstudio.com/items?itemName=eamodio.gitlens)

   * [Checkstyle for Java](https://marketplace.visualstudio.com/items?itemName=shengchen.vscode-checkstyle)

----

## Git Setup
Perform the following steps in bash shell (e.g. git bash on windows
or any terminal window on linux or macos):

1. Identify yourself

   This is so the repository knows who made various changes.
   We should have this set in our private repositories.

   Identify who you are so that git commits can be tagged and
   the changes are traceable back to the person who made them.
   Do not use shared accounts as a rule of thumb.

   **Warning**: In public repositories there may be some privacy concerns
   exposing names and contact information, particularly of minors.

   Our repositories should be private, but they could potentially become
   public and the identities in the history are hard (if not impossible)
   ot cleanse.

   If you do not want to expose your identify then "<your name>" would
   be the identity that your registered as under github and leave your
   email out. However if you do this then other members of the team
   (including mentors) are not going to know who you are. We should have
   some private document that maps team members with their git id's so
   that we can know who is who and be able to trace back changes.

       git config --global user.name "<your github id>"

   or (see **Warning** above)

       git config --global user.name "<your name>"
       git config --global user.email "<your email>"

   Set [end-of-line handling](https://docs.github.com/en/get-started/getting-started-with-git/configuring-git-to-handle-line-endings)

    | Platform | Command |
    |----------|---------|
    | Windows  | `git config --global core.autocrlf true`  |
    | MacOs    | `git config --global core.autocrlf input` |
    | Linux    | `git config --global core.autocrlf input` |


1. Test your git configuration
       mkdir test
       cd test
       touch file
       git init
       git add .
       git commit -a -m "test"
       git log

   Confirm the `Author` line is correct

1. Remove the temporary repository from previouis step

       cd ..
       rm -rf test

1. Setup Git credentials
   You might want to configure a credential store so you dont need to 
   keep typing your password. Use one of:

   | Mechanism  |  Command
   |------------|----------
   | [Forever](https://git-scm.com/docs/git-credential-store)  | `git config --global credential.helper store`
   | [15 minutes](https://git-scm.com/docs/git-credential-cache) | `git config --global credential.helper cache`
   | [N seconds](https://git-scm.com/docs/git-credential-cache) | `git config --global credential.helper 'cache --timeout=N'`


1. Test your credentials

       git clone https://github.com/FRC-Team-1403/TemplateRobotRepository.git
       rm -rf TemplateRobotRepository

   We wont need that repository. We just wanted to verify your credentials can
   access our private repositories.

   **See one of the Programming Team captains if you do not have access**.

   ----
   