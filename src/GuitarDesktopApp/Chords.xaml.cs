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

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for Chords.xaml
    /// </summary>
    public partial class Chords : Page
    {
        public Chords()
        {
            InitializeComponent();
        }

        private void UI_Home_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.Navigate(new Page1());
        }

        private void UI_HowTo_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.Navigate(new HowToReadChord());
        }
    }
}
