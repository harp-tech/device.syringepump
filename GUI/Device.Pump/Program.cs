using Avalonia;
using System;
using Device.Pump.GUI;

namespace Device.Pump
{
    internal class Program
    {
        // Initialization code. Don't use any Avalonia, third-party APIs or any
        // SynchronizationContext-reliant code before AppMain is called: things aren't initialized
        // yet and stuff might break.
        [STAThread]
        public static void Main(string[] args)
        {
            BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);

            StartApp.CloseLog();
        }
        
        public static AppBuilder BuildAvaloniaApp()
        {
            return StartApp.BuildAvaloniaApp();
        }
    }
}
