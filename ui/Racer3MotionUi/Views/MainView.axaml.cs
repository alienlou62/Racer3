using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Racer3MotionUi.ViewModels;

namespace Racer3MotionUi.Views;

public partial class MainView : UserControl
{
    private Key? _activeJogKey;

    public MainView()
    {
        InitializeComponent();
        Focusable = true;
    }

    private MainViewModel? ViewModel => DataContext as MainViewModel;

    private async void OnJogZMinusPointerPressed(object? sender, PointerPressedEventArgs e)
    {
        if (ViewModel is not { } viewModel)
        {
            return;
        }

        Focus();

        if (sender is IInputElement inputElement)
        {
            e.Pointer.Capture(inputElement);
        }

        await viewModel.StartBackendCartesianJogAsync("Z-", "pointer press");
        e.Handled = true;
    }

    private async void OnJogPointerReleased(object? sender, PointerReleasedEventArgs e)
    {
        if (sender is IInputElement)
        {
            e.Pointer.Capture(null);
        }

        if (ViewModel is { } viewModel)
        {
            await viewModel.StopBackendCartesianJogAsync("pointer release");
        }

        e.Handled = true;
    }

    private async void OnKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Source is TextBox)
        {
            return;
        }

        if (ViewModel is not { } viewModel)
        {
            return;
        }

        if (e.Key is Key.Escape or Key.Space)
        {
            _activeJogKey = null;
            await viewModel.StopBackendCartesianJogAsync(e.Key == Key.Escape ? "escape key" : "space key");
            e.Handled = true;
            return;
        }

        if (!IsZMinusJogKey(e.Key))
        {
            return;
        }

        if (_activeJogKey == e.Key)
        {
            e.Handled = true;
            return;
        }

        if (_activeJogKey.HasValue && _activeJogKey.Value != e.Key)
        {
            await viewModel.StopBackendCartesianJogAsync("new jog key pressed");
        }

        _activeJogKey = e.Key;
        await viewModel.StartBackendCartesianJogAsync("Z-", $"key down {e.Key}");
        e.Handled = true;
    }

    private async void OnKeyUp(object? sender, KeyEventArgs e)
    {
        if (e.Source is TextBox)
        {
            return;
        }

        if (!IsZMinusJogKey(e.Key))
        {
            return;
        }

        if (_activeJogKey == e.Key)
        {
            _activeJogKey = null;
        }

        if (ViewModel is { } viewModel)
        {
            await viewModel.StopBackendCartesianJogAsync($"key release {e.Key}");
        }

        e.Handled = true;
    }

    private async void OnLostFocus(object? sender, RoutedEventArgs e)
    {
        if (!ReferenceEquals(e.Source, this))
        {
            return;
        }

        _activeJogKey = null;

        if (ViewModel is { IsBackendJogActive: true } viewModel)
        {
            await viewModel.StopBackendCartesianJogAsync("lost focus");
        }
    }

    private static bool IsZMinusJogKey(Key key)
    {
        return key is Key.S or Key.Down;
    }
}
