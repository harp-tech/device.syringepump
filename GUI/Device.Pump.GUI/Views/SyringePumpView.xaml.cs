using Avalonia.Markup.Xaml;
using Avalonia.ReactiveUI;
using Device.Pump.GUI.ViewModels;

namespace Device.Pump.GUI.Views
{
    public class SyringePumpView : ReactiveUserControl<SyringePumpViewModel>
    {
        public SyringePumpView()
        {
            InitializeComponent();
        }

        private void InitializeComponent()
        {
            AvaloniaXamlLoader.Load(this);
        }
    }
}
