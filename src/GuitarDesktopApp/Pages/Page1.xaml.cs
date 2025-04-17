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
using GuitarDesktopApp.Pages;
using System.Net.Sockets;
using System.Net;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for Page1.xaml
    /// </summary>
    public partial class Page1 : Page
    {
        //properties
        //fields 
        public MainWindow mainWin; //main window object to hold variables that can be accessed by all pages
        
        public Page1(MainWindow mainWindow)
        {
            InitializeComponent();
            DataContext = mainWindow;
            mainWin = mainWindow;
            mainWin.udpClient = new UdpClient();
        }

        private void UI_Chord_Btn_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                byte[] sendBuffer = Encoding.UTF8.GetBytes("Chords.");
                mainWin.udpClient.Send(sendBuffer, sendBuffer.Length);
            }
            catch (Exception ex)
            {
                Trace.WriteLine($"Could not send : {ex.Message}");
            }
            NavigationService.Navigate(new Chords(mainWin));
        }

        private async void UI_Connect_Btn_Click(object sender, RoutedEventArgs e)
        {
            Trace.WriteLine("Connect button clicked");

            try
            {

                mainWin.udpClient.Connect("172.20.10.12", 1666);
                byte[] sendBuffer = Encoding.UTF8.GetBytes("hello pico");
                mainWin.udpClient.Send(sendBuffer, sendBuffer.Length);
                Trace.WriteLine("Hello pico was sent.");
            }
            catch (Exception ex)
            {
                Trace.WriteLine("Something went wrong");
                return;
            }
        }

        private void UI_Scales_Btn_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                byte[] sendBuffer = Encoding.UTF8.GetBytes("Scales.");
                mainWin.udpClient.Send(sendBuffer, sendBuffer.Length);
                Trace.WriteLine("Scales was sent.");
            }
            catch (Exception ex)
            {
                Trace.WriteLine($"Could not send : {ex.Message}");
            }
            NavigationService.Navigate(new Scales(mainWin));
        }
    }
}
