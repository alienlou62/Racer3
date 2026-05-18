using Avalonia.Controls;
using Avalonia.Input;
using Racer3MotionUi.ViewModels;

namespace Racer3MotionUi.Views;

public partial class MainView : UserControl
{
    public MainView()
    {
        InitializeComponent();
        AttachedToVisualTree += (_, _) => Focus();
        KeyDown += OnKeyDown;
    }

    private async void OnKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Source is TextBox)
        {
            return;
        }

        if (DataContext is not MainViewModel viewModel || !viewModel.IsJogModeEnabled)
        {
            return;
        }

        var keyName = e.Key.ToString();

        switch (e.Key)
        {
            case Key.Up:
            case Key.Down:
            case Key.Left:
            case Key.Right:
            case Key.W:
            case Key.S:
            case Key.A:
            case Key.D:
            case Key.Q:
            case Key.E:
                e.Handled = true;
                await viewModel.HandleJogKeyAsync(keyName);
                break;

            case Key.Space:
                e.Handled = true;
                if (viewModel.StopSessionMotionCommand.CanExecute(null))
                {
                    await viewModel.StopSessionMotionCommand.ExecuteAsync(null);
                }
                break;
        }
    }
}
