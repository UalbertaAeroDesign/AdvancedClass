# Creating the Virtual Environment:
1) Create a folder somewhere on your computer to host your SITL virtual environment
2) Open your terminal/command prompt
3) cd into the folder you initially created for SITL
4) Run the following command: python -m venv venv
   You have now successfuly created your virtual environment. Now to enter it!
5) Run the following command: venv\Scripts\activate
   If you are on Windows Powershell, run this: venv\Scripts\Activate.ps1
   If you are getting an error that indicates "unauthorized access", run these commands in order:
    - Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
    - venv\Scripts\Activate.ps1
   If successful, you will see (venv) before your current path
6) Download the requirements.txt file from the github repo and copy it onto the location of your virtual environment
7) In your virtual environment location, run the following command: pip install -r requirements.txt
    As long as there are no errors, you have successfully installed the packages you need.




1) Install WSL for windows. It should be as simple as running the following command in powershell and then restarting your computer: wsl --install
2) Search "ubunut" in the start menu and open the application
3) Within the application, run these three commands (Press Y+Enter if asked to continue):
   - sudo apt-get update
   - sudo apt-get install git
   - sudo apt-get install gitk git-gui
4) Clone ArduPilot's repo by running the following command:
   - git clone "https://github.com/ArduPilot/ardupilot.git"
5) Enter the ArduPilot repository by entering the following command:
   - cd ardupilot
6) Run the following commands (this is a script that ArduPilot provides that will install some required packages):
   - Tools/environment_install/install-prereqs-ubuntu.sh -y
   - . ~/.profile
7) Exit Ubuntu, go into windows powershell, and type the following command:
   - wsl --shutdown
8) Repoen Ubuntu, if there's a path in brackets before your actual path, type:
   - deactivate


# Starting The SITL simulator:

1) Type the following command:
   - cd ardupilot
2) Run the following command:
   - ./waf configure --board sitl && ./waf plane
   - if the output says you need to install a package, run the following command:
      - sudo apt install python3-<package_name>
      - run: ./waf configure --board sitl && ./waf plane
3) Run the following commands:
   - sudo apt update && sudo apt upgrade -y
   - sudo apt install python3-pip python3-dev python3-setuptools python3-wheel -y
   - pip install matplotlib --break-system-packages
   - pip install pymavlink MAVProxy --break-system-packages
   - echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
   - . ~/.bashrc
4) run the following command:
   - cd ArduCopter
5) run the following command:
   - ../Tools/autotest/sim_vehicle.py --map --console
   If you get an error saying "No module named 'future'", run this command and then try the above command again:
      - pip install future --break-system-packages
   