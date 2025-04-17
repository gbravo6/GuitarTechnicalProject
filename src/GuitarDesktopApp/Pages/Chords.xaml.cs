using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.Sockets;
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
using Newtonsoft.Json;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for Chords.xaml
    /// </summary>
    public partial class Chords : Page
    {
        //fields and properties
        MainWindow mainWin = null;
        public Chords(MainWindow mainWindow)
        {
            InitializeComponent();
            mainWin = mainWindow;
        }

        private void UI_Home_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.GoBack();
        }

        private void UI_HowTo_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.Navigate(new HowToReadChord());
        }

        private void A_btn_Click(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-A.");
        }
        private void Am_btn_Clicked(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-Am.");
        }

        private void C_btn_Click(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-C.");
        }

        private void D_btn_Click(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-D.");
        }

        private void Dm_btn_Click(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-Dm.");
        }

        private void e_btn_Click(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-E.");
        }

        private void em_btn_click(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-Em.");
        }

        private void g_btn_click(object sender, RoutedEventArgs e)
        {
            UDPSend("Ch-G.");
        }

        private void UDPSend(string s)
        {
            try
            {
                mainWin.udpClient.Send(Encoding.UTF8.GetBytes(s));
                Trace.WriteLine($"Send Success: {s}");
            }
            catch (Exception ex)
            {
                Trace.WriteLine($"Send failed : {ex.Message}");
            }
        }
    }
}
