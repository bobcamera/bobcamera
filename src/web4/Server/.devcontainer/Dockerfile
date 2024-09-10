FROM mcr.microsoft.com/dotnet/sdk:8.0

# install node via nodesource ppa
# https://github.com/nodesource/distributions#debinstall

RUN apt-get update -y 
RUN apt-get install curl -y

# install template
RUN dotnet new install Clean.Architecture.Solution.Template::8.0.5

# install dev-certs
RUN dotnet dev-certs https
