using System.Diagnostics;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using InTheHand.Net.Bluetooth;
using InTheHand.Net.Sockets;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //fields 
        BluetoothClient btc = null;
        BluetoothDeviceInfo[] devices = null;
        public MainWindow()
        {
            InitializeComponent();
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

        private void ToPage1_Btn_Click(object sender, RoutedEventArgs e)
        {
            Content = new Page1();
        }
    }
}