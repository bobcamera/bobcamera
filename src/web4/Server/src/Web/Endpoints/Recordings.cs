using bob_web4.Application.Common.Models;
using bob_web4.Application.Recordings.Commands.DeleteRecording;
using bob_web4.Application.Recordings.Queries.GetRecordings;

namespace bob_web4.Web.Endpoints;

public class Recordings : EndpointGroupBase
{
    public override void Map(WebApplication app)
    {
        app.MapGroup(this)
            //.RequireAuthorization()
            .MapGet(GetRecordings)
            .MapDelete(DeleteRecording, "{file}");
    }

    public Task<List<RecordingDto>> GetRecordings(ISender sender, [AsParameters] GetRecordingsQuery query)
    {
        return sender.Send(query);
    }

    public async Task<IResult> DeleteRecording(ISender sender, string file)
    {
        await sender.Send(new DeleteRecordingCommand(file));
        return Results.NoContent();
    }
}
