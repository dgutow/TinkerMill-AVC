#!/usr/bin/python
"""
 printOut.py - prints followed by a flush

 Written by David Gutow 3/2018
"""

import sys

DIAGNOSTICS = True      # Output these diagnostics?

###############################################################################
def printOut(string):
    if DIAGNOSTICS:
        print (string)
        sys.stdout.flush()
# end     

###############################################################################
def printErr(string):
    if DIAGNOSTICS:
        print >>sys.stderr, (string)
        sys.stderr.flush()
# end   

###############################################################################
def printDiagnostics (output=True):
    DIAGNOSTICS = output
# end

###############################################################################