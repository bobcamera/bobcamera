using bob_web4.Application.Common.Interfaces;
using bob_web4.Domain.Events;

namespace bob_web4.Application.Recordings.Commands.DeleteRecording;

public record DeleteRecordingCommand(string recording) : IRequest;

public class DeleteRecordingCommandHandler : IRequestHandler<DeleteRecordingCommand>
{
    private readonly IApplicationDbContext _context;

    public DeleteRecordingCommandHandler(IApplicationDbContext context)
    {
        _context = context;
    }

    public async Task Handle(DeleteRecordingCommand request, CancellationToken cancellationToken)
    {
        /*var entity = await _context.TodoItems
            .FindAsync(new object[] { request.Id }, cancellationToken);

        Guard.Against.NotFound(request.Id, entity);

        _context.TodoItems.Remove(entity);

        entity.AddDomainEvent(new TodoItemDeletedEvent(entity));

        await _context.SaveChangesAsync(cancellationToken);*/

        await Task.CompletedTask;
    }
}
