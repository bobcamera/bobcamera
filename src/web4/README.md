# bob_web4

The project was generated using the [Clean.Architecture.Solution.Template](https://github.com/jasontaylordev/CleanArchitecture) version 8.0.5.

## Build

Run `dotnet build -tl` to build the solution.

## Run

To run the web application:

```bash
cd .\src\Web\
dotnet watch run
```

Navigate to https://localhost:5001. The application will automatically reload if you change any of the source files.

## Code Styles & Formatting

The template includes [EditorConfig](https://editorconfig.org/) support to help maintain consistent coding styles for multiple developers working on the same project across various editors and IDEs. The **.editorconfig** file defines the coding styles applicable to this solution.

## Code Scaffolding

The template includes support to scaffold new commands and queries.

Start in the `.\src\Application\` folder.

Create a new command:

```
dotnet new ca-usecase --name CreateTodoList --feature-name TodoLists --usecase-type command --return-type int
```

Create a new query:

```
dotnet new ca-usecase -n GetTodos -fn TodoLists -ut query -rt TodosVm
```

If you encounter the error *"No templates or subcommands found matching: 'ca-usecase'."*, install the template and try again:

```bash
dotnet new install Clean.Architecture.Solution.Template::8.0.5
```

## Test

The solution contains unit, integration, functional, and acceptance tests.

To run the unit, integration, and functional tests (excluding acceptance tests):
```bash
dotnet test --filter "FullyQualifiedName!~AcceptanceTests"
```

To run the acceptance tests, first start the application:

```bash
cd .\src\Web\
dotnet run
```

Then, in a new console, run the tests:
```bash
cd .\src\Web\
dotnet test
```

## Help
To learn more about the template go to the [project website](https://github.com/jasontaylordev/CleanArchitecture). Here you can find additional guidance, request new features, report a bug, and discuss the template with other users.


## Migrations

### Install dotnet-ef
```bash
dotnet tool install --global dotnet-ef --version 8.*
```

### List existing migrations
```bash
/root/.dotnet/tools/dotnet-ef migrations list --project ./src/Infrastructure --startup-project ./src/Web
```

### Add a new migration called RecordingMigration
```bash
/root/.dotnet/tools/dotnet-ef migrations add "RecordingMigration" --project ./src/Infrastructure --startup-project ./src/Web --output-dir Data/Migrations
```

### Undo migrations from the database up to InitialCreate i.e. only have InitialCreate applied
```bash
/root/.dotnet/tools/dotnet-ef database update "00000000000000_InitialCreate" --project ./src/Infrastructure --startup-
project ./src/Web
```

### Remove migrations from the project
```bash
/root/.dotnet/tools/dotnet-ef migrations remove --project ./src/Infrastructure --startup-project ./src/Web
```