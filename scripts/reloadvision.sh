sudo /home/cpetit/bin/unloadvision.sh
if [ ! $? = 0 ]; then
 echo "[WARNING] Cannot remove properly modules"
fi
sudo /home/cpetit/bin/loadvision.sh
