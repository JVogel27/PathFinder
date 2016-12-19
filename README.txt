Project: Orienteering in Mendon Ponds Park
Author: Jesse Vogel
Date Created: September 13th, 2016

Description: Give the program a file containing necessary event details and the program will generate a map with the best 
path drawn in red, (named map.png.) It will also generate a text file that describes the path in plain english, (named directions.txt.)

Two Event Types:
In a classic event, the file is represented by a simple text file, two integers per line, representing the (x,y) 
pixel (origin at upper left) in the terrain map containing the location. In a Score-O, you are given a start/finish 
location and a set of controls to visit. You may visit the controls in any order, but must return to the finish in 
a given amount of time. A Score-O file will have a first line saying "ScoreO", a second line with the allowed time in 
seconds, a third line with the location of the start/finish, and then additional lines representing the control locations.

An example input file for classic and score-o is labeled classic_input.txt score_o_input.txt

To run the program:
1. place program.py, terrain.png, and mpp.txt into an empty directory.
2. In the same directory, place your input file to pass to the program, (classic_input.txt or score_o_input.txt.)
3. run the command 'python program.py inputfile.txt'
4. view results in map.png and directions.txt
