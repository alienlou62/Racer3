using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class GeminiMotionChatService : IMotionChatService
{
    private const string GenerateContentBaseUrl = "https://generativelanguage.googleapis.com/v1beta/models";

    private readonly HttpClient _httpClient;
    private readonly string? _apiKey;
    private readonly string _model;

    public GeminiMotionChatService(Racer3MotionUiConfig config, HttpClient httpClient)
    {
        _httpClient = httpClient;
        _apiKey = Environment.GetEnvironmentVariable("GEMINI_API_KEY");
        _model = ResolveValue(Environment.GetEnvironmentVariable("GEMINI_MODEL"), config.GeminiModel);
    }

    public string StatusText => IsAvailable
        ? $"Gemini mode: {_model}"
        : "Gemini unavailable: GEMINI_API_KEY is not set. Using local command parser.";

    public bool IsAvailable => !string.IsNullOrWhiteSpace(_apiKey);

    public async Task<MotionChatResponse> InterpretAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        if (!IsAvailable)
        {
            throw new MotionChatProviderUnavailableException("GEMINI_API_KEY is not set.");
        }

        var payload = new
        {
            systemInstruction = new
            {
                parts = new object[]
                {
                    new
                    {
                        text = MotionChatJson.SystemPrompt
                    }
                }
            },
            contents = new object[]
            {
                new
                {
                    role = "user",
                    parts = new object[]
                    {
                        new
                        {
                            text = MotionChatJson.BuildUserPrompt(request)
                        }
                    }
                }
            },
            generationConfig = new
            {
                temperature = 0.0,
                responseMimeType = "application/json"
            }
        };

        var endpoint = $"{GenerateContentBaseUrl}/{Uri.EscapeDataString(_model)}:generateContent?key={Uri.EscapeDataString(_apiKey!)}";
        using var httpRequest = new HttpRequestMessage(HttpMethod.Post, endpoint);
        httpRequest.Content = new StringContent(
            JsonSerializer.Serialize(payload),
            Encoding.UTF8,
            "application/json");

        using var httpResponse = await _httpClient.SendAsync(httpRequest, cancellationToken);
        var responseBody = await httpResponse.Content.ReadAsStringAsync();

        if (!httpResponse.IsSuccessStatusCode)
        {
            throw new MotionChatServiceException(
                $"Gemini planner request failed with HTTP {(int)httpResponse.StatusCode}.");
        }

        var content = ExtractAssistantContent(responseBody);
        return MotionChatJson.ParseMotionChatResponse(content, request.CurrentPlan);
    }

    private static string ExtractAssistantContent(string responseBody)
    {
        try
        {
            using var document = JsonDocument.Parse(responseBody);
            var candidates = document.RootElement.GetProperty("candidates");
            if (candidates.GetArrayLength() == 0)
            {
                throw new MotionChatInvalidResponseException("Gemini returned no candidates.");
            }

            var parts = candidates[0].GetProperty("content").GetProperty("parts");
            if (parts.GetArrayLength() == 0)
            {
                throw new MotionChatInvalidResponseException("Gemini returned no content parts.");
            }

            var content = parts[0].GetProperty("text").GetString();
            if (string.IsNullOrWhiteSpace(content))
            {
                throw new MotionChatInvalidResponseException("Gemini returned an empty response.");
            }

            return content;
        }
        catch (JsonException exception)
        {
            throw new MotionChatInvalidResponseException("Could not parse Gemini generateContent response.", exception);
        }
        catch (KeyNotFoundException exception)
        {
            throw new MotionChatInvalidResponseException("Gemini response did not contain assistant content.", exception);
        }
        catch (InvalidOperationException exception)
        {
            throw new MotionChatInvalidResponseException("Gemini response content had an unexpected shape.", exception);
        }
    }

    private static string ResolveValue(string? environmentValue, string configValue)
    {
        return string.IsNullOrWhiteSpace(environmentValue)
            ? configValue
            : environmentValue.Trim();
    }
}
