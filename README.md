# Introduction
TODO: Give a short introduction of your project. Let this section explain the objectives or the motivation behind this project. 

This project is aimed to perform simulations of the magnetic behavior of single domain magnetic nanoparticles for magnetic particle imaging.

<hr>

### Getting Started

</details>

TODO: Guide users through getting your code up and running on their own system. In this section you can talk about:

<details>
<summary>Installation prerequisite</summary>

 1. [CMake](https://cmake.org/) 
 2. [MATLAB](www.mathworks.com)
 3. C++ compiler e.g. clang(-cl) from [LLVM](https://llvm.org/) (don't use MSVC if you care about performance)
 
</details>

<details>
<summary>Installation process</summary>

From a cmd line:
 1. clone/checkout the git repo. 
 2. (optional) setup vcpkg
 3. configure with cmake e.g. `cmake -G "Ninja" -S <path-to-source> -B <path-to-build>` (in source builds are not allowed! Meaning <path-to-build> is not allowed to be within <path-to-source>)
 4. build/install with cmake e.g. `cmake --build [<options>] -B <path-to-build>`
 
</details>

<details>
<summary>Running test</summary>

From a cmd line:
 1. clone/checkout the git repo. 
 2. (optional) setup vcpkg
 3. configure with cmake e.g. `cmake -G "Ninja" -S <path-to-source> -B <path-to-build>` (no in source builds!)
 4. build/install with cmake e.g. `cmake --build [<options>] -S <path-to-source> -B <path-to-build>`
 
</details>

<details>
<summary>Running benchmarks</summary>

From a cmd line:
 1. clone/checkout the git repo. 
 2. (optional) setup vcpkg
 3. configure with cmake e.g. `cmake -G "Ninja" -S <path-to-source> -B <path-to-build>` (no in source builds!)
 4. build/install with cmake e.g. `cmake --build [<options>] -S <path-to-source> -B <path-to-build>`
 
</details>
1.	
For UNIX like systems:
Also download/clone pcg, boost, eigen & cereal and but in the folder C:\Path\Extern\ or change all include paths within the project to the correct folders.
Also put the MATLAB bin and includes in the Folder C:\Path\Extern\
If you want to run the programm make sure C:\Program Files\Matlab\<MATLABVERSION>\bin\win64 is part of the PATH variable

2.	Software dependencies
Matlab (required)
Boost (optional)
PCG-Random (optional)
Cereal (optional)
HDF5 (optional)

sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get install gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 70 
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 70 
sudo apt install build-essential

Install/Update latest CMake
(sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget)
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update
sudo apt-get install kitware-archive-keyring
sudo apt-key --keyring /etc/apt/trusted.gpg del C1F34CDD40CD72DA
sudo apt-get install cmake
Install/Update latest llvm
bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 10
sudo apt-get install clang-11 clang-tools-11 clang-11-doc libclang-common-11-dev libclang-11-dev libclang1-11 clang-format-11 clangd-11 lld-11 lldb-11
sudo apt-get install libc++-11-dev libc++abi-11-dev libc++-dev libc++1 libc++abi-dev libc++abi1
sudo apt-get install libllvm-11-ocaml-dev libllvm11 llvm-11 llvm-11-dev llvm-11-doc llvm-11-examples llvm-11-runtime

sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-11 70 
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-11 70 
3.	Latest releases
No official release yet, still under development


4.	API references

#Build and Test
TODO: Describe and show how to build your code and run the tests. 

#Contribute
TODO: Explain how other users and developers can contribute to make your code better. 

Create your own branch und create push requests.


If you want to learn more about creating good readme files then refer the following [guidelines](https://www.visualstudio.com/en-us/docs/git/create-a-readme). You can also seek inspiration from the below readme files:
- [ASP.NET Core](https://github.com/aspnet/Home)
- [Visual Studio Code](https://github.com/Microsoft/vscode)
- [Chakra Core](https://github.com/Microsoft/ChakraCore)
