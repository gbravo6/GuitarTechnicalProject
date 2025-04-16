using System;
using System.Collections.Generic;
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

namespace GuitarDesktopApp.Pages
{
    /// <summary>
    /// Interaction logic for Scales.xaml
    /// </summary>
    public partial class Scales : Page
    {
        MainWindow mainWin = null;
        public Scales(MainWindow mW)
        {
            InitializeComponent();
            mainWin = mW;
        }

        private void UI_Home_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.GoBack();
        }

        private void em_pentatonic_click(object sender, RoutedEventArgs e)
        {
            mainWin.udpClient.SendAsync(Encoding.UTF8.GetBytes("Sc-Em"));
        }
    }
}
