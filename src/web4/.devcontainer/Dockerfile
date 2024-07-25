FROM mcr.microsoft.com/dotnet/sdk:8.0

# install node via nodesource ppa
# https://github.com/nodesource/distributions#debinstall

RUN apt-get update -y 
RUN apt-get install curl -y

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
RUN apt-get install -y nodejs

# install template
RUN dotnet new install Clean.Architecture.Solution.Template::8.0.5

# install dev-certs
RUN dotnet dev-certs https

# install Angular cli
RUN npm install -g @angular/cli@15.2.8