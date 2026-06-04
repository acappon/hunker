#!/usr/bin/env python3

import subprocess
import sqlite3

db=sqlite3.connect("dbase/lg.sqlite")

c=db.cursor()

c.execute("select file_name from lg")

names = c.fetchall()

for n in names:
   name = n[0]
   with open("HTML/{0}.html".format(name), "w") as outfile:
      subprocess.run(["bin/html.py", name], stdout=outfile, check=True)
   print(name)

c.close()

db.close()

