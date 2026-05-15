using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class OllamaMotionChatService : IMotionChatService
{
    private readonly HttpClient _httpClient;
    private readonly string _baseUrl;
    private readonly string _model;

    public OllamaMotionChatService(Racer3MotionUiConfig config, HttpClient httpClient)
    {
        _httpClient = httpClient;
        _baseUrl = NormalizeBaseUrl(Environment.GetEnvironmentVariable("OLLAMA_BASE_URL"), config.OllamaBaseUrl);
        _model = ResolveValue(Environment.GetEnvironmentVariable("OLLAMA_MODEL"), config.OllamaModel);
    }

    public string StatusText => $"Ollama mode: {_model} at {_baseUrl}";

    public bool IsAvailable => true;

    public async Task<MotionChatResponse> InterpretAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        var payload = new
        {
            model = _model,
            stream = false,
            format = "json",
            options = new
            {
                temperature = 0.0
            },
            messages = new object[]
            {
                new
                {
                    role = "system",
                    content = MotionChatJson.SystemPrompt
                },
                new
                {
                    role = "user",
                    content = MotionChatJson.BuildUserPrompt(request)
                }
            }
        };

        using var httpRequest = new HttpRequestMessage(HttpMethod.Post, $"{_baseUrl}/api/chat");
        httpRequest.Content = new StringContent(
            JsonSerializer.Serialize(payload),
            Encoding.UTF8,
            "application/json");

        HttpResponseMessage httpResponse;
        try
        {
            httpResponse = await _httpClient.SendAsync(httpRequest, cancellationToken);
        }
        catch (HttpRequestException exception)
        {
            throw new MotionChatProviderUnavailableException(
                $"Ollama unavailable at {_baseUrl}.",
                exception);
        }
        catch (TaskCanceledException exception) when (!cancellationToken.IsCancellationRequested)
        {
            throw new MotionChatProviderUnavailableException(
                $"Ollama unavailable at {_baseUrl}.",
                exception);
        }

        using (httpResponse)
        {
            var responseBody = await httpResponse.Content.ReadAsStringAsync();

            if (!httpResponse.IsSuccessStatusCode)
            {
                throw new MotionChatProviderUnavailableException(
                    $"Ollama unavailable at {_baseUrl}. HTTP {(int)httpResponse.StatusCode}.");
            }

            var content = ExtractAssistantContent(responseBody);
            return MotionChatJson.ParseMotionChatResponse(content, request.CurrentPlan);
        }
    }

    private static string ExtractAssistantContent(string responseBody)
    {
        try
        {
            using var document = JsonDocument.Parse(responseBody);
            var content = document.RootElement
                .GetProperty("message")
                .GetProperty("content")
                .GetString();

            if (string.IsNullOrWhiteSpace(content))
            {
                throw new MotionChatInvalidResponseException("Ollama returned an empty response.");
            }

            return content;
        }
        catch (JsonException exception)
        {
            throw new MotionChatInvalidResponseException("Could not parse Ollama chat response.", exception);
        }
        catch (KeyNotFoundException exception)
        {
            throw new MotionChatInvalidResponseException("Ollama response did not contain assistant content.", exception);
        }
        catch (InvalidOperationException exception)
        {
            throw new MotionChatInvalidResponseException("Ollama response content had an unexpected shape.", exception);
        }
    }

    private static string NormalizeBaseUrl(string? environmentValue, string configValue)
    {
        var rawValue = ResolveValue(environmentValue, configValue);
        return rawValue.TrimEnd('/');
    }

    private static string ResolveValue(string? environmentValue, string configValue)
    {
        return string.IsNullOrWhiteSpace(environmentValue)
            ? configValue
            : environmentValue.Trim();
    }
}
