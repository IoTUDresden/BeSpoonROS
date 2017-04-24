#!/usr/bin/env python 

import subprocess

def runJava(jarname):
    print("Java process is calling....")
    proc = subprocess.Popen(['java', '-jar', jarname], stdout=subprocess.PIPE)
    print("Process id: " , proc.pid)
    while proc.poll() is None: 
        output = proc.stdout.readline()
        print(output.strip())
    retcode = proc.poll()
    print("Return code: ", retcode);
    print("Java process finished")

if __name__ == "__main__":
    runJava('bespoon.jar')
