using System;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class MotionChatProviderRouter : IMotionChatService
{
    private readonly IMotionChatService _localRules;
    private readonly IMotionChatService _ollama;
    private readonly IMotionChatService _openAi;
    private readonly string _requestedProvider;
    private string _statusText;

    public MotionChatProviderRouter(
        Racer3MotionUiConfig config,
        IMotionChatService localRules,
        IMotionChatService ollama,
        IMotionChatService openAi)
    {
        _localRules = localRules;
        _ollama = ollama;
        _openAi = openAi;
        _requestedProvider = ResolveRequestedProvider(config);
        _statusText = CreateInitialStatusText();
    }

    public string StatusText => _statusText;

    public bool IsAvailable => true;

    public async Task<MotionChatResponse> InterpretAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        return _requestedProvider switch
        {
            "openai" => await TryProviderWithLocalFallbackAsync(_openAi, "OpenAI", request, cancellationToken),
            "ollama" => await TryProviderWithLocalFallbackAsync(_ollama, "Ollama", request, cancellationToken),
            "auto" => await InterpretAutoAsync(request, cancellationToken),
            "local" or "localrules" or "rules" => await UseLocalRulesAsync(request, cancellationToken),
            _ => await UseUnknownProviderFallbackAsync(request, cancellationToken)
        };
    }

    private async Task<MotionChatResponse> InterpretAutoAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        if (_openAi.IsAvailable)
        {
            return await TryProviderWithLocalFallbackAsync(_openAi, "OpenAI", request, cancellationToken);
        }

        return await UseLocalRulesAsync(request, cancellationToken);
    }

    private async Task<MotionChatResponse> TryProviderWithLocalFallbackAsync(
        IMotionChatService provider,
        string providerName,
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        if (!provider.IsAvailable)
        {
            _statusText = $"{providerName} unavailable: {ProviderUnavailableReason(providerName)} Using local command parser.";
            return await _localRules.InterpretAsync(request, cancellationToken);
        }

        _statusText = provider.StatusText;
        try
        {
            return await provider.InterpretAsync(request, cancellationToken);
        }
        catch (MotionChatProviderUnavailableException exception)
        {
            _statusText = $"{exception.Message} Using local command parser.";
            return await _localRules.InterpretAsync(request, cancellationToken);
        }
    }

    private async Task<MotionChatResponse> UseLocalRulesAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        _statusText = _localRules.StatusText;
        return await _localRules.InterpretAsync(request, cancellationToken);
    }

    private async Task<MotionChatResponse> UseUnknownProviderFallbackAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        _statusText = $"Unknown chat provider '{_requestedProvider}'. Using local command parser.";
        return await _localRules.InterpretAsync(request, cancellationToken);
    }

    private string CreateInitialStatusText()
    {
        return _requestedProvider switch
        {
            "openai" => _openAi.IsAvailable
                ? _openAi.StatusText
                : "OpenAI unavailable: OPENAI_API_KEY is not set. Using local command parser.",
            "ollama" => _ollama.StatusText,
            "auto" => _openAi.IsAvailable ? _openAi.StatusText : _localRules.StatusText,
            "local" or "localrules" or "rules" => _localRules.StatusText,
            _ => $"Unknown chat provider '{_requestedProvider}'. Using local command parser."
        };
    }

    private static string ResolveRequestedProvider(Racer3MotionUiConfig config)
    {
        var providerFromEnvironment = Environment.GetEnvironmentVariable("RACER3_CHAT_PROVIDER");
        if (!string.IsNullOrWhiteSpace(providerFromEnvironment))
        {
            return NormalizeProvider(providerFromEnvironment);
        }

        if (!string.IsNullOrWhiteSpace(Environment.GetEnvironmentVariable("OPENAI_API_KEY")))
        {
            return "openai";
        }

        return NormalizeProvider(config.ChatProvider);
    }

    private static string NormalizeProvider(string provider)
    {
        return provider.Trim().ToLowerInvariant().Replace("_", string.Empty).Replace("-", string.Empty);
    }

    private static string ProviderUnavailableReason(string providerName)
    {
        return providerName == "OpenAI"
            ? "OPENAI_API_KEY is not set."
            : "provider is not available.";
    }
}
