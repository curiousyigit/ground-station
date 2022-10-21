# Drone Controller
This is a Python 3 based drone controller built within **Shoaib Feda**'s bachelors graduation project at the **Polytechnic Institute of Tomar**, Portugal under the supervision of **Professor Gabriel Pires**
<br></br>
# Getting started
## Assumptions
- You have **Python 3.8** installed with the command `python -V` already pointing to it without the need to specify `python3 -V`. Otherwise, all commands within this file should be modified accordingly. The commands are mainly for **Linux Ubuntu** based operating systems, modify accordingly if you are using anything different
## Requirements
- Python 3.8 virtual environment with all dependencies in `requirements.txt`. You can use `pip install -r requirements.txt`
## Activate Virtual Environment & Run Program
- Activate the virtual environment using `source venv/bin/activate`
- Read the `README.md` inside the desired mode to learn more
- Run `python main.py --drone=tello --mode=face`
- Press `q` on the keyboard to quit
- To deactivate the virtual environment, use the `deactivate` command
<br></br>
# How to make changes
- Modify the code as required
- Freeze dependencies using `python -m pip freeze > requirements.txt`
- Commit to git