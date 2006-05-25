import os

# Run the Doxygen application to generate the documentation.
os.system("rd /q/s \"Output/Doxygen\"")
os.system("mkdir \"Output/Doxygen\"")
os.system("doxygen.exe Doxyfile")

# Generate a second index file which simply points to the inner one.
indexFile = open("Output/Doxygen/FCollada.html", "w")
indexFile.write("<title>FCollada Documentation</title>")
indexFile.write("<frameset><frame src=\"html/index.html\"></frameset>")
indexFile.close()

# Open the Doxygeon log file to users so they know what's missing.
os.system("write \"Output/Doxygen/log.txt\"")
