   M S B A M L   `   `   `  ����N  JRobotControlLibrary, Version=1.0.0.0, Culture=neutral, PublicKeyToken=nullV>clr-namespace:RobotControlLibrary;assembly=RobotControlLibraryRobotControlLibrary  5      G CRobot_window, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null;"clr-namespace:Robot_window.UI_PageRobot_window.UI_Page 5      (   "Robot_window.UI_Page.JointMovePage   5      W SPresentationCore, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35e aSystem.Windows.Controls.Ribbon, Version=6.0.2.0, Culture=neutral, PublicKeyToken=b77a5c561934e089R NWindowsBase, Version=6.0.2.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     5      R NSystem.Xaml, Version=6.0.2.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5      Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5      @d2http://schemas.microsoft.com/expression/blend/2008     5      NRCL>clr-namespace:RobotControlLibrary;assembly=RobotControlLibrary     5      4local"clr-namespace:Robot_window.UI_Page     5        N� Title$  JointMovePage��5      .��5      � .+�5   
   *   $RobotControlLibrary.RobotMoveContorl  -   5          Name$ Joint1��5      $��0��5      $��	59,34,0,0q�5         IntervalTime$ 30��5         IncrementValue$	 0.5Q�5      5        5   
   -   5        $ Joint2��5      $��0��5      $��	59,84,0,0q�5      $ 30��5      $	 0.5Q�5      5        5   
   -   5        $ Joint3��5      $��0��5      $��
59,134,0,0q�5      $ 30��5      $	 0.5Q�5      5        5   
   -   5        $ Joint4��5"      $��0��5      $��
59,184,0,0q�5       $ 30��5!      $	 0.5Q�5#      5$        5%   
   -   5        $ Joint5��5(      $��0��5%      $��
59,234,0,0q�5&      $ 30��5'      $	 0.5Q�5)      5*        5+   
   -   5        $ Joint6��5.      $��0��5+      $��
59,284,0,0q�5,      $ 30��5-      $	 0.5Q�5/      50      �� 51   
   -   5         ��Name$ speed��51      $	��150��52       � Minimum$ 1Q�53       � Maximum$ 80Q�54      $
��Left=�55      $	��Top=�56       � SmallChange$ 1Q�58      $��
51,349,0,0q�59      $��0��5:      �� 5;   
   $��30��6   $��速度��5<      $��34��5=      $	��Top=�5>      $
��Left=�5?      $��	5,344,0,0q�5@      $��0��5A      �� 5B   
   -   5         ��Name$ 	speedshow��5B      $��30��5C      $��0%��5D      $��45��5E      $	��Top=�5F      $
��Left=�5G      $��200,344,0,0q�5H      5J      5L      