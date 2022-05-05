#/bin/bash -e

# This file generates a fresh batch of test cases in results.csv at the root
# of the project.

echo "rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 1 --env cafeWorld --file-name results.csv --clean"
rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 1 --env cafeWorld --file-name results.csv --clean

echo "rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 2 --env cafeWorld --file-name results.csv"
rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 2 --env cafeWorld --file-name results.csv

echo "rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 3 --env cafeWorld --file-name results.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 3 --env cafeWorld --file-name results.csv

echo "rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 4 --env cafeWorld --file-name results.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 4 --env cafeWorld --file-name results.csv

echo "rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 5 --env bookWorld --file-name results.csv"
rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 5 --env bookWorld --file-name results.csv

echo "rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 6 --env bookWorld --file-name results.csv"
rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 6 --env bookWorld --file-name results.csv

echo "rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 7 --env bookWorld --file-name results.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 7 --env bookWorld --file-name results.csv

echo "rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 8 --env bookWorld --file-name results.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 8 --env bookWorld --file-name results.csv
