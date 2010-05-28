sudo ./unloadvision.sh
if [ ! $? = 0 ]; then
 echo "[WARNING] Cannot remove properly modules"
fi
sudo ./loadvision.sh
