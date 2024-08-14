using bob_web4.Application.Common.Models;
using bob_web4.Application.Vision.Queries.GetCamera;

namespace bob_web4.Web.Endpoints;

public class Vision : EndpointGroupBase
{
    public override void Map(WebApplication app)
    {
        app.MapGroup(this)
            //.RequireAuthorization()
            .MapGet(GetCamera);

    }

    public Task<CameraDto> GetCamera(ISender sender, [AsParameters] GetCameraQuery query)
    {
        return sender.Send(query);
    }
}
