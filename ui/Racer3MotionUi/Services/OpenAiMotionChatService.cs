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

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true
    };

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
        ? $"LLM mode: {_model}"
        : "LLM unavailable: OPENAI_API_KEY is not set. Using local command parser.";

    public bool IsAvailable => !string.IsNullOrWhiteSpace(_apiKey);

    public async Task<MotionChatResponse> InterpretAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        if (!IsAvailable)
        {
            throw new MotionChatServiceException("OPENAI_API_KEY is not set.");
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
                    content = SystemPrompt
                },
                new
                {
                    role = "user",
                    content = BuildUserPrompt(request)
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
        return ParseMotionChatResponse(content, request.CurrentPlan);
    }

    private static string SystemPrompt =>
        "You are a robot motion planning assistant for a Comau Racer3 demo UI.\n" +
        "You only produce safe preview plans. You never execute robot motion.\n" +
        "Return strict JSON only. No markdown.\n" +
        "Allowed shapes: circle, square, triangle, hexagon.\n" +
        "Allowed actions: update_plan, reset_plan, explain, reject.\n" +
        "Motion is drawn in a constant-X Y/Z plane.\n" +
        "You may update shape, centerX, centerY, centerZ, sizeMeters, velocity, and cornerMode.\n" +
        "You must always set canExecute=false.\n" +
        "If the user asks to run or execute, explain that the operator must validate, arm Confirm Motion, and press Run Selected Shape manually.\n" +
        "Keep values conservative.\n" +
        "Use this JSON shape exactly: {\"action\":\"update_plan\",\"shape\":\"square\",\"centerX\":0.5,\"centerY\":0.0,\"centerZ\":-0.55,\"sizeMeters\":0.05,\"velocity\":0.04,\"cornerMode\":\"sharp\",\"assistantMessage\":\"I prepared a safe preview plan. Please validate before live motion.\",\"canExecute\":false}";

    private static string BuildUserPrompt(MotionChatRequest request)
    {
        var prompt = new
        {
            currentPlan = ToPromptPlan(request.CurrentPlan),
            defaultPlan = ToPromptPlan(request.DefaultPlan),
            userCommand = request.UserCommand,
            safetyBounds = new
            {
                centerX = "0.40 to 0.60",
                centerY = "-0.15 to 0.15",
                centerZ = "-0.65 to -0.45",
                sizeMeters = "0.02 to 0.08",
                velocity = "0.02 to 0.08"
            }
        };

        return JsonSerializer.Serialize(prompt);
    }

    private static object ToPromptPlan(MotionPlan plan)
    {
        return new
        {
            shape = plan.Shape.ToString().ToLowerInvariant(),
            centerX = plan.CenterX,
            centerY = plan.CenterY,
            centerZ = plan.CenterZ,
            sizeMeters = plan.SizeMeters,
            velocity = plan.Velocity,
            cornerMode = plan.CornerMode
        };
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

    private static MotionChatResponse ParseMotionChatResponse(string json, MotionPlan currentPlan)
    {
        LlmMotionPlanResponse? parsed;
        try
        {
            parsed = JsonSerializer.Deserialize<LlmMotionPlanResponse>(json, JsonOptions);
        }
        catch (JsonException exception)
        {
            throw new MotionChatInvalidResponseException("LLM returned invalid JSON. The current plan was not changed.", exception);
        }

        if (parsed == null)
        {
            throw new MotionChatInvalidResponseException("LLM returned an empty JSON object. The current plan was not changed.");
        }

        var action = ParseAction(parsed.Action);
        var assistantMessage = string.IsNullOrWhiteSpace(parsed.AssistantMessage)
            ? "I prepared a preview plan. Please validate before live motion."
            : parsed.AssistantMessage.Trim();

        if (action is MotionChatAction.Explain or MotionChatAction.Reject)
        {
            return new MotionChatResponse(action, assistantMessage, canExecute: false);
        }

        if (action == MotionChatAction.ResetPlan)
        {
            return new MotionChatResponse(action, assistantMessage, canExecute: false);
        }

        if (!TryParseShape(parsed.Shape, currentPlan.Shape, out var shape))
        {
            return new MotionChatResponse(
                MotionChatAction.Reject,
                "The planner returned an unsupported shape, so I left the current plan unchanged.",
                canExecute: false);
        }

        var plan = new MotionPlan
        {
            Shape = shape,
            CenterX = parsed.CenterX ?? currentPlan.CenterX,
            CenterY = parsed.CenterY ?? currentPlan.CenterY,
            CenterZ = parsed.CenterZ ?? currentPlan.CenterZ,
            SizeMeters = parsed.SizeMeters ?? currentPlan.SizeMeters,
            Velocity = parsed.Velocity ?? currentPlan.Velocity,
            CornerMode = string.IsNullOrWhiteSpace(parsed.CornerMode)
                ? currentPlan.CornerMode
                : parsed.CornerMode.Trim()
        };

        return new MotionChatResponse(MotionChatAction.UpdatePlan, assistantMessage, plan, canExecute: false);
    }

    private static MotionChatAction ParseAction(string? action)
    {
        return action?.Trim().ToLowerInvariant() switch
        {
            "update_plan" => MotionChatAction.UpdatePlan,
            "reset_plan" => MotionChatAction.ResetPlan,
            "explain" => MotionChatAction.Explain,
            "reject" => MotionChatAction.Reject,
            _ => MotionChatAction.Reject
        };
    }

    private static bool TryParseShape(string? value, ShapeKind fallback, out ShapeKind shape)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            shape = fallback;
            return true;
        }

        return value.Trim().ToLowerInvariant() switch
        {
            "circle" => SetShape(ShapeKind.Circle, out shape),
            "square" => SetShape(ShapeKind.Square, out shape),
            "triangle" => SetShape(ShapeKind.Triangle, out shape),
            "hexagon" => SetShape(ShapeKind.Hexagon, out shape),
            _ => FailShape(out shape)
        };
    }

    private static bool SetShape(ShapeKind value, out ShapeKind shape)
    {
        shape = value;
        return true;
    }

    private static bool FailShape(out ShapeKind shape)
    {
        shape = ShapeKind.Circle;
        return false;
    }

    private sealed class LlmMotionPlanResponse
    {
        public string? Action { get; init; }

        public string? Shape { get; init; }

        public double? CenterX { get; init; }

        public double? CenterY { get; init; }

        public double? CenterZ { get; init; }

        public double? SizeMeters { get; init; }

        public double? Velocity { get; init; }

        public string? CornerMode { get; init; }

        public string? AssistantMessage { get; init; }

        public bool CanExecute { get; init; }
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
