using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class OpenAiMotionChatService : IMotionChatService
{
    private const string ChatCompletionsUrl = "https://api.openai.com/v1/chat/completions";

    private readonly HttpClient _httpClient;
    private readonly string? _apiKey;
    private readonly string _model;

    public OpenAiMotionChatService(Racer3MotionUiConfig config, HttpClient httpClient)
    {
        _httpClient = httpClient;
        _apiKey = Environment.GetEnvironmentVariable("OPENAI_API_KEY");
        var modelFromEnvironment = Environment.GetEnvironmentVariable("OPENAI_MODEL");
        _model = string.IsNullOrWhiteSpace(modelFromEnvironment)
            ? config.OpenAiModel
            : modelFromEnvironment;
    }

    public string StatusText => IsAvailable
        ? $"OpenAI mode: {_model}"
        : "LLM unavailable: OPENAI_API_KEY is not set. Using local command parser.";

    public bool IsAvailable => !string.IsNullOrWhiteSpace(_apiKey);

    public async Task<MotionChatResponse> InterpretAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        if (!IsAvailable)
        {
            throw new MotionChatProviderUnavailableException("OPENAI_API_KEY is not set.");
        }

        var payload = new
        {
            model = _model,
            temperature = 0.0,
            response_format = new
            {
                type = "json_object"
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

        using var httpRequest = new HttpRequestMessage(HttpMethod.Post, ChatCompletionsUrl);
        httpRequest.Headers.Authorization = new AuthenticationHeaderValue("Bearer", _apiKey);
        httpRequest.Content = new StringContent(
            JsonSerializer.Serialize(payload),
            Encoding.UTF8,
            "application/json");

        using var httpResponse = await _httpClient.SendAsync(httpRequest, cancellationToken);
        var responseBody = await httpResponse.Content.ReadAsStringAsync();

        if (!httpResponse.IsSuccessStatusCode)
        {
            throw new MotionChatServiceException(
                $"OpenAI planner request failed with HTTP {(int)httpResponse.StatusCode}.");
        }

        var content = ExtractAssistantContent(responseBody);
        return MotionChatJson.ParseMotionChatResponse(content, request.CurrentPlan);
    }

    private static string ExtractAssistantContent(string responseBody)
    {
        try
        {
            using var document = JsonDocument.Parse(responseBody);
            var choices = document.RootElement.GetProperty("choices");
            if (choices.GetArrayLength() == 0)
            {
                throw new MotionChatInvalidResponseException("LLM returned no choices.");
            }

            var content = choices[0].GetProperty("message").GetProperty("content").GetString();
            if (string.IsNullOrWhiteSpace(content))
            {
                throw new MotionChatInvalidResponseException("LLM returned an empty response.");
            }

            return content;
        }
        catch (JsonException exception)
        {
            throw new MotionChatInvalidResponseException("Could not parse OpenAI chat completion response.", exception);
        }
        catch (KeyNotFoundException exception)
        {
            throw new MotionChatInvalidResponseException("OpenAI response did not contain assistant content.", exception);
        }
        catch (InvalidOperationException exception)
        {
            throw new MotionChatInvalidResponseException("OpenAI response content had an unexpected shape.", exception);
        }
    }

}

public class MotionChatServiceException : Exception
{
    public MotionChatServiceException(string message)
        : base(message)
    {
    }

    public MotionChatServiceException(string message, Exception innerException)
        : base(message, innerException)
    {
    }
}

public sealed class MotionChatInvalidResponseException : MotionChatServiceException
{
    public MotionChatInvalidResponseException(string message)
        : base(message)
    {
    }

    public MotionChatInvalidResponseException(string message, Exception innerException)
        : base(message, innerException)
    {
    }
}

public sealed class MotionChatProviderUnavailableException : MotionChatServiceException
{
    public MotionChatProviderUnavailableException(string message)
        : base(message)
    {
    }

    public MotionChatProviderUnavailableException(string message, Exception innerException)
        : base(message, innerException)
    {
    }
}
