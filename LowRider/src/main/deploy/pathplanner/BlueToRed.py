#!/usr/bin/python3

import os, sys, getopt

FieldWidth = 16.503396 # inches: 652.73 - 3.0

def convert_x(x):
   return FieldWidth - x

def convert_rotation(rotation):
   return -1 * rotation

def produce_output(zlist, outputfile):
      with open(outputfile, 'w') as fo:
         fo.writelines(zlist)
         # add CR/LF
         fo.write("\n")

def parse_path(inputfile):
   plist = []

   with open(inputfile) as fi:
      lines = fi.readlines()

   for linenum in range(len(lines)):
      line = lines[linenum]

      if line.__contains__('"x"'):
         parts = line.split('"x": ')
         number = parts[1].split(',')[0]
         zx = convert_x(float(number))
         plist.append(parts[0] + '"x": ' + str(zx) + ',' + '\n')
      elif line.__contains__('"rotation"'):
         rparts = line.split('"rotation": ')
         rnumber = rparts[1].split(',')[0]
         zr = convert_rotation(float(rnumber))
         plist.append(rparts[0] + '"rotation": ' + str(zr) + ',' + '\n')
      elif line.__contains__('"folder"'):
         fline = line.replace("Blu", "Red")
         plist.append(fline)
      else:
         plist.append(line)

   return plist

def parse_auto(inputfile):
   zlist = []

   with open(inputfile) as fi:
      lines = fi.readlines()

   for linenum in range(len(lines)):
      line = lines[linenum]

      if line.__contains__('"pathName"'):
         zlist.append(line.replace("Blu", "Red"))

         # dirname = os.path.dirname(inputfile)
         # filename = os.path.basename(inputfile)
         # filename = filename.replace("Blu", "Red")
         # outputPath = os.path.join(dirname, filename)
         # print(outputPath)

         pathParts = line.split('"')
         fileName = pathParts[3].split('.')[0]
         pathName = os.path.join("paths", fileName + ".path")
         print('In  File: ' + pathName)
         outfilePath = os.path.join("paths", "Red" + fileName.split('Blu')[1]  + ".path")
         print('Out File: ' + outfilePath)
         plist = parse_path(pathName)
         produce_output(plist, outfilePath)
      elif line.__contains__('"x"'):
         parts = line.split('"x": ')
         number = parts[1].split(',')[0]
         zx = convert_x(float(number))
         zlist.append(parts[0] + '"x": ' + str(zx) + ',' + '\n')
      else:
         zlist.append(line)

   return zlist

def main(argv):
   inputfile = None
   outputfile = None
   try:
      opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
   except getopt.GetoptError:
      print ('BlueToRed.py -i <inputfile> -o <outputfile>')
      sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print ('BlueToRed.py -i <inputfile> -o <outputfile>')
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg
   if inputfile:
      if not outputfile:
         dirname = os.path.dirname(inputfile)
         filename = os.path.basename(inputfile)
         filename = filename.replace("Blu", "Red")
         outputfile = os.path.join(dirname, filename)

      print ('Input file is:  ', inputfile)
      print ('Output file is: ', outputfile)
      zlist = parse_auto(inputfile)
      produce_output(zlist, outputfile)
   else:
      print("Nothing Done.")

if __name__ == "__main__":
   main(sys.argv[1:])