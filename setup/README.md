
# **Setup**

This is a simple guide to set up your local or remote machines with proper configurations.

## Aliases

To setup custom aliases used for this project you need to create a symbolic link that points to `.bash_aliases_m1` file located in this directory. Follow the list of instructions below and enter the commands on your terminal. **(NOTE: Make sure remote repo is cloned in the appropriate location! Repo should be cloned in your home directory.)**

1. Assuming this repo was cloned in your home directory, navigate to your home directory using the command: 
   - `cd`.
2. Now create a soft link using this command: 
   - `ln -s $HOME/Mock1/setup/.bash_aliases_m1 .bash_aliases_m1`
3. Next, open the your `.bashrc` file, located in your home directory, using your favorite editor and add the following to the bottom of the file with proper syntax as shown below:<br>
![image](https://user-images.githubusercontent.com/91917978/172930918-9a40f921-7794-4523-b7de-5079c96fc3e3.png)
4. Finally, open a new tab and you should be able to use the custom aliases located in the `~/Mock1/setup/.bash_aliases_m1` file.
