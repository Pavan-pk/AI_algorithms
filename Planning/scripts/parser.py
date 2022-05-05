#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import argparse
import os
import time
import utils

# Create the argument parser.
parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument("--objtypes",
    default=1,
    type=int,
    help="No of object types")
    
parser.add_argument("--objcount",
    default=1,
    type=int,
    help="The number of objects for each type in the environment.")
    
parser.add_argument("--seed",
    default=int(time.time()),
    type=int,
    help="The random seed")
    
parser.add_argument("--generate-only",
    default=False,
    action="store_true",
    help="Only generate a new problem file and exit.")
    
parser.add_argument("--file-name",
    default="results.csv",
    help="Store results in <file> in the project root directory.")

parser.add_argument("--clean",
    default=False,
    action="store_true",
    help="Cleanup the existing csv files")

parser.add_argument("--env",
    default="bookWorld",
    type=str,
    help="Choose environment b/w cafeWorld and bookWorld")

def parse_args():
    """
        Parses the cmd line arguments.

        Returns
        ========
            args: Namespace
                The parsed args.

        Raises
        =======
            ArgumentError
                If the arguments do not parse correctly.
    """
    args = parser.parse_args()

    if args.env not in ["cafeWorld","bookWorld"]:
        raise ValueError("Environment can only be one of 'cafeWorld' or 'bookWorld'")

    if args.objtypes > 2:
        raise Exception('Maximum no. of objects available is: 2')

    elif args.objtypes < 1:
        raise Exception('Minimum no. of objects available is: 1')

    if args.objcount > 3:
        raise Exception('Maximum no. of objects available for each type is: 3')
        
    elif args.objcount < 1:
        raise Exception('Minimum no. of objects available for each type is: 1')

    return args
