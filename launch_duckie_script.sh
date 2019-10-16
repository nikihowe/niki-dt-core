# cd to the package directory (in my case, 'rosdev' is an alias for doing that).
cd /Users/niki/udem-fall19-public/catkin_ws/src/niki-dt-core/

BRANCH_NAME="v1"
DOCKER_USERNAME="nikihowe"
GIT_REPO="niki-dt-core"
DEMO_NAME="pure_pursuit"
PACKAGE_NAME="pure_pursuit"
DUCKIEBOT_NAME="chloe"

timeout_delay=120

# add, commit, and push the recent changes (commit -s -v will open an editor window if there are any changes)   
git add .
git checkout $BRANCH_NAME
git commit -s -v -m "Run on duckie"
git push

# push the docker image to dockerhub
# NOTE: I had to edit out some command.py files in dts source for this to work, since I don't seem to be able to push to the duckietown repository.
# ALSO: I kept the same base image as before in the dockerfile ("FROM duckietown/${BASE_IMAGE}:${BASE_TAG}")
# (the files I edited were:
# ~/.dt-shell/commands-multi/daffy/devel/build/command.py, line 96
# ~/.dt-shell/commands-multi/daffy/devel/push/command.py, lines 68, 69 and 74
# (I replaced 'duckietown' with my docker username.)

# EDIT: added this loop that kills and re-attempts if the push takes longer than 3 minutes. 
dts devel build --push

echo "removing image from duckiebot"
docker -H $DUCKIEBOT_NAME.local rmi $DOCKER_USERNAME/$GIT_REPO:$BRANCH_NAME

echo "Pulling Image onto the duckiebot"
docker -H $DUCKIEBOT_NAME.local pull $DOCKER_USERNAME/$GIT_REPO:$BRANCH_NAME

echo "Running demo on duckiebot"
dts duckiebot demo --demo_name $DEMO_NAME --package_name $PACKAGE_NAME --duckiebot_name $DUCKIEBOT_NAME --image $DOCKER_USERNAME/$GIT_REPO:$BRANCH_NAME
