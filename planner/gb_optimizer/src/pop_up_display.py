#!/usr/bin/env python3

import tkinter as tk


class PopUpDisplay():
    def __init__(self):
        self.user_input = 0.0
        self.entry = 0.0  # Declare entry as an instance variable
        self.ONOFF=False
        self.INTEGER=False
    def get_user_input(self):
        try:
            self.user_input = float(self.entry.get())  # Try converting user input to float
            print("user_input:", self.user_input)
            self.window.destroy()  # Close the window
        except ValueError:
            # Exception handling for invalid input (not a valid float)
            print("Invalid input! Please enter a valid number.")
    
    def get_user_input_binary(self):
        self.user_input = self.entry.get()
        try:
            if self.user_input in ('0', '1'):
                self.user_input = int(self.user_input)  # Convert user input to binary
                print("user_input(binary):", self.user_input)
                self.window.destroy()  # Close the window
            else:
                self.user_input = int(self.entry.get())
                print("Warning: Input must be 0 or 1.")
        
        except ValueError:
            # Exception handling for invalid input (not a valid float)
            print("Invalid input! Please enter a valid number.")
    
    
    def get_user_input_integer(self):
        self.user_input = self.entry.get()
        try:
            if int(self.user_input)>0:
                self.user_input = int(self.user_input)  # Convert user input to int
                print("user_input(integer):", self.user_input)
                self.window.destroy()  # Close the window
            else:
                print("Warning: Input must be integer.")
        
        except ValueError:
            # Exception handling for invalid input (not a valid float)
            print("Invalid input! Please enter a valid number.")

    def show_input_dialog(self,menu):
        self.window = tk.Tk()
        self.window.title(menu+" User Input")

        # Get the screen dimensions
        screen_width = self.window.winfo_screenwidth()
        screen_height = self.window.winfo_screenheight()

        # Calculate the window coordinates for centering
        window_width = 300
        window_height = 100
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2

        # Set the window size and position
        self.window.geometry(f"{window_width}x{window_height}+{x}+{y}")

        label = tk.Label(self.window, text="Enter a "+menu+":")
        label.pack()

        self.entry = tk.Entry(self.window)
        self.entry.pack()

        if self.ONOFF:
            button = tk.Button(self.window, text="Submit", command=self.get_user_input_binary)
        elif self.INTEGER:
            button = tk.Button(self.window, text="Submit", command=self.get_user_input_integer)
        else:
            button = tk.Button(self.window, text="Submit", command=self.get_user_input)
        button.pack()

        self.window.mainloop()