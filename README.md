# Ground Station
This is a Python 3 based drone ground station built within **Shoaib Feda**'s bachelors graduation project at the **Polytechnic Institute of Tomar**, Portugal under the supervision of **Professor Gabriel Pires**
<br></br>
# Getting started
## Assumptions
- You have **Python 3.8** installed with the command `python -V` already pointing to it without the need to specify `python3 -V`. Otherwise, all commands within this file should be modified accordingly. The commands are mainly for **Linux Ubuntu** based operating systems, modify accordingly if you are using anything different
- You need git installed in your system to be able to explore the different branches of this repository
## Requirements
- Python 3.8 virtual environmentwith all dependencies in `requirements.txt`. You can use `pip install -r requirements.txt`
## Activate Virtual Environment & Run Program
- Activate the virtual environment using `source venv/bin/activate` (skip if you don't use virtual environments)
- Read the `README.md` inside the desired mode to learn more
- Run `python main.py --drone=tello --mode=face`
- Press `q` on the keyboard to quit
- To deactivate the virtual environment, use the `deactivate` command (skip if you don't use virtual environments)
<br></br>
# How to make changes
- Modify the code as required
- Update requirements.txt, for example by using the python library `pipreqs` using the command `pipreqs --force`. Becareful however that this library is buggy and may miss some libraries used in your code
- Commit to git