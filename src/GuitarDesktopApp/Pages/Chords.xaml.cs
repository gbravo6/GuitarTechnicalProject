using System;
using System.Collections.Generic;
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

        private async void A_btn_Click(object sender, RoutedEventArgs e)
        {
            //SendJSON("A");
            mainWin.udpClient.Send(Encoding.UTF8.GetBytes("ch_A-"),"ch_A".Length);
        }

        public async void SendJSON(string chord)
        {
            //prep and send through socket 
            var jsonSend = new
            {
                chord = chord
            };

            byte[] sendBuffer = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(jsonSend));
            await mainWin._Client.SendAsync(new ArraySegment<byte>(sendBuffer), SocketFlags.None);
        }
    }
}
