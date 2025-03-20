using InTheHand.Net.Sockets;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for Page1.xaml
    /// </summary>
    public partial class Page1 : Page
    {
        //properties
        //fields 
        BluetoothClient btc = null;
        BluetoothDeviceInfo[] devices = null;
        public Page1()
        {
            InitializeComponent();
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            MainWindow home = new MainWindow();
            NavigationService.Navigate(home);

        }
        private void UI_Scan_Btn_Click(object sender, RoutedEventArgs e)
        {
            UI_Devices_List.Items.Clear();
            //scan for devices
            try
            {

                btc = new BluetoothClient();
                devices = btc.DiscoverDevices();
            }
            catch (Exception ex)
            {
                Trace.WriteLine($"Scanning Devices Failed: {ex.Message}");
                UI_Devices_List.Items.Add("Scanning Devices Failed.");
            }
            foreach (BluetoothDeviceInfo device in devices)
            {
                UI_Devices_List.Items.Add($"Device Name: {device.DeviceName} | Address: {device.DeviceAddress}");
            }
        }

        private void UI_Chord_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.Navigate(new Chords());
        }
    }
}
