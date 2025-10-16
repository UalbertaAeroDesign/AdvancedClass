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
6)  