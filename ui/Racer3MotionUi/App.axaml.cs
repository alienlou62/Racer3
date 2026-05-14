using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Data.Core;
using Avalonia.Data.Core.Plugins;
using Avalonia.Markup.Xaml;
using Racer3MotionUi.Services;
using Racer3MotionUi.ViewModels;
using Racer3MotionUi.Views;

namespace Racer3MotionUi;

public partial class App : Application
{
    public override void Initialize()
    {
        AvaloniaXamlLoader.Load(this);
    }

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            // Line below is needed to remove Avalonia data validation.
            // Without this line you will get duplicate validations from both Avalonia and CT
            ExpressionObserver.DataValidators.RemoveAll(x => x is DataAnnotationsValidationPlugin);
            var config = Racer3MotionUiConfig.Load();
            var processRunner = new ProcessRunner();
            var motionService = new PowerShellRobotMotionService(config, processRunner);
            var shapePathPlanner = new ShapePathPlanner();

            desktop.MainWindow = new MainWindow
            {
                DataContext = new MainViewModel(shapePathPlanner, motionService, config),
            };
        }

        base.OnFrameworkInitializationCompleted();
    }
}
