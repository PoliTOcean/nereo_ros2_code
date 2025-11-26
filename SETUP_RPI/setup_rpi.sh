sudo apt-get update
sudo apt-get upgrade -y

RED='\033[0;31m'
NC='\033[0m' # No Color
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
# current working directory
cwd=$(pwd)

echo "Installing Git..."
sudo apt-get install git -y
echo "${GREEN}Done.${NC}"

echo "Installing Vim..."
sudo apt-get install vim -y
echo "${GREEN}Done.${NC}"

echo "Installing Midnight Commander..."
sudo apt-get install mc -y
echo "${GREEN}Done.${NC}"

echo "Installing Ranger..."
sudo apt-get install ranger -y
echo "${GREEN}Done.${NC}"

echo "Installing fzf..."
sudo apt-get install fzf -y
echo "${GREEN}Done.${NC}"

echo "Installing htop..."
sudo apt-get install htop -y
echo "${GREEN}Done.${NC}"

echo "Installing thefu*k..."
sudo apt-get install thefuck -y
echo "${GREEN}Done.${NC}"

echo "Installing tmux..."
sudo apt-get install tmux -y
echo "${GREEN}Done.${NC}"

echo "Configuring tmux..."

echo "Installing tmux tpm..."
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
echo "${GREEN}Done.${NC}"
echo "${YELLOW}"
mkdir -p ~/.config/tmux
cp ./tmux.conf ~/.config/tmux/

~/.tmux/plugins/tpm/bin/install_plugins
~/.tmux/plugins/tpm/bin/update_plugins all
echo "${NC}"
echo "${GREEN}Done.${NC}"

echo "Installing neovim..."
echo "Installing dependencies..."
sudo apt-get install ninja-build gettext cmake unzip curl build-essential -y
echo "${YELLOW}"
git clone https://github.com/neovim/neovim.git ~/neovim
cd ~/neovim && make CMAKE_BUILD_TYPE=RelWithDebInfo
cd build && cpack -G DEB && sudo dpkg -i nvim-linux64.deb

echo "${GREEN}Done.${NC}"

echo "Installing nvchad..."
git clone https://github.com/NvChad/starter ~/.config/nvim
echo -e "${RED}Done! Run :MasonInstallAll after nvchad is done installing."
echo -e "Then delete ~/.config/nvim/.git${NC}"

echo "Installing ROS2 Humble..."

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "${GREEN}Done.${NC}"

echo "alias rg="ranger "" >> ~/.bashrc

echo "Building Nereo ROS2 packages..."
cd $cwd/../rpi_ws
colcon build

echo "${RED}All set up!${NC}"
