using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Threading.Tasks;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;

namespace Robot_window
{
    public class PortViewModel : INotifyPropertyChanged
    {
        private ObservableCollection<string> _availablePorts = new ObservableCollection<string>();
        public ObservableCollection<string> AvailablePorts
        {
            get => _availablePorts;
            set { _availablePorts = value; OnPropertyChanged(); }
        }

        // 刷新端口列表
        public void RefreshPorts()
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                AvailablePorts.Clear();
                foreach (var port in System.IO.Ports.SerialPort.GetPortNames())
                {
                    AvailablePorts.Add(port);
                }
            });
        }

        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
