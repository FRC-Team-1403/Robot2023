Setting up Windows Subsystem for Linux (WSL)
---

# Why

WSL runs an actual unix distribution (Ubuntu) allowing you to run
Linux commands and programs within Windows on the same filesystem.
The bash shell in particular is very useful and we have several
scripts that make life easier. These can be run in a WSL shell
even if you do everything else in Windows.

It is ok if you do not know Linux. The main motivation for having
it is to follow instructions that already write the specific commands.


# Install WSL

Follow [microsoft's install instructions](https://docs.microsoft.com/en-us/windows/wsl/install)

# Look around

Run WSL to get a shell.

Make sure you installed version 20.04. If not then these instructions might
be out of date.
`lsb_release -rs`

Your home directory in this environment is going to be `/home/$USER`
but in a different filesystem than Windows.  The windows filesystems
will be mounted under /mnt (e.g. your Windows home directory will
be `/mnt/c/Users/$USER`

You can see all the filesystems using the standard unix `df`
(e.g. `df -kh` to see the information more human-readable)

For convenience make a symbolic link from your Linux home directory
to your Windows home directory. This will make it easy to navigate
from Linux to the Windows directory where your workspace is.

```bash
cd $HOME
ln -s /mnt/c/users/$USER windows
```

now you can use `$HOME/windows` to refer to `/mnt/c/users/$USER`

# Install Packages

1. Add [microsoft's package manager](https://docs.microsoft.com/en-us/java/openjdk/install#install-on-ubuntu)
```bash
ubuntu_release=`lsb_release -rs`
wget https://packages.microsoft.com/config/ubuntu/${ubuntu_release}/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
```

1. Add OpenJdk

   Note that this is JDK 17, not JDK 11. I think that is OK.
```bash
sudo apt-get install apt-transport-https
sudo apt-get update
sudo apt-get install msopenjdk-17
```

1. Confirm it is installed

   `java -version`



