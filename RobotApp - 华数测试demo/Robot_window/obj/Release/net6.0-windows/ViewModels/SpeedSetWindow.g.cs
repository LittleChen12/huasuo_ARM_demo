﻿#pragma checksum "..\..\..\..\ViewModels\SpeedSetWindow.xaml" "{ff1816ec-aa5e-4d10-87f7-6f4963833460}" "982D697D8FA62BDDB3EDCBD0DA20B4FCE89E5A39"
//------------------------------------------------------------------------------
// <auto-generated>
//     此代码由工具生成。
//     运行时版本:4.0.30319.42000
//
//     对此文件的更改可能会导致不正确的行为，并且如果
//     重新生成代码，这些更改将会丢失。
// </auto-generated>
//------------------------------------------------------------------------------

using Robot_window.ViewModels;
using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Controls.Ribbon;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;


namespace Robot_window.ViewModels {
    
    
    /// <summary>
    /// SpeedSetWindow
    /// </summary>
    public partial class SpeedSetWindow : System.Windows.Window, System.Windows.Markup.IComponentConnector {
        
        
        #line 18 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox jointspeed;
        
        #line default
        #line hidden
        
        
        #line 31 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox XYZspeed;
        
        #line default
        #line hidden
        
        
        #line 44 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox ABCspeed;
        
        #line default
        #line hidden
        
        
        #line 57 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox c2183speed;
        
        #line default
        #line hidden
        
        
        #line 70 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox jointintelvalue;
        
        #line default
        #line hidden
        
        
        #line 83 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox XYZintelvalue;
        
        #line default
        #line hidden
        
        
        #line 96 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox ABCintelvalue;
        
        #line default
        #line hidden
        
        
        #line 107 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox Tick;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "9.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/Robot_window;component/viewmodels/speedsetwindow.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "9.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            this.jointspeed = ((System.Windows.Controls.TextBox)(target));
            return;
            case 2:
            this.XYZspeed = ((System.Windows.Controls.TextBox)(target));
            return;
            case 3:
            this.ABCspeed = ((System.Windows.Controls.TextBox)(target));
            return;
            case 4:
            this.c2183speed = ((System.Windows.Controls.TextBox)(target));
            return;
            case 5:
            this.jointintelvalue = ((System.Windows.Controls.TextBox)(target));
            return;
            case 6:
            this.XYZintelvalue = ((System.Windows.Controls.TextBox)(target));
            return;
            case 7:
            this.ABCintelvalue = ((System.Windows.Controls.TextBox)(target));
            return;
            case 8:
            this.Tick = ((System.Windows.Controls.TextBox)(target));
            return;
            case 9:
            
            #line 120 "..\..\..\..\ViewModels\SpeedSetWindow.xaml"
            ((System.Windows.Controls.Button)(target)).Click += new System.Windows.RoutedEventHandler(this.SpeedSetClick);
            
            #line default
            #line hidden
            return;
            }
            this._contentLoaded = true;
        }
    }
}

