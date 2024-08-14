namespace bob_web4.Application.Vision.Queries.GetCamera;

public class GetCameraQueryValidator : AbstractValidator<GetCameraQuery>
{
    public GetCameraQueryValidator()
    {
        RuleFor(x => x.CameraName)
            .NotEmpty().WithMessage("CameraName is required.");
    }
}
