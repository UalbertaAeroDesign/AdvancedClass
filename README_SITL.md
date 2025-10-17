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