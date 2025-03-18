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
    /// Interaction logic for HowToReadChord.xaml
    /// </summary>
    public partial class HowToReadChord : Page
    {
        public HowToReadChord()
        {
            InitializeComponent();
        }

        private void UI_Back_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.GoBack();
        }
    }
}
