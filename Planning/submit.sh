#/bin/bash -e

# Do not change the parameters in this file.

echo "rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 111 --env cafeWorld --file-name submit.csv --clean"
rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 111 --env cafeWorld --file-name submit.csv --clean
echo "rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 222 --env cafeWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 222 --env cafeWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 333 --env cafeWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 333 --env cafeWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 444 --env cafeWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 444 --env cafeWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 2 --objcount 3 --seed 555 --env cafeWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 3 --seed 555 --env cafeWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 111 --env bookWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 111 --env bookWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 222 --env bookWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 1 --objcount 2 --seed 222 --env bookWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 333 --env bookWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 1 --seed 333 --env bookWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 444 --env bookWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 2 --seed 444 --env bookWorld --file-name submit.csv
echo "rosrun hw2 refinement.py --objtypes 2 --objcount 3 --seed 555 --env bookWorld --file-name submit.csv"
rosrun hw2 refinement.py --objtypes 2 --objcount 3 --seed 555 --env bookWorld --file-name submit.csv