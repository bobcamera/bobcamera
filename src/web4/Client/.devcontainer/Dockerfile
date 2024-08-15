FROM mcr.microsoft.com/dotnet/sdk:8.0

# install node via nodesource ppa
# https://github.com/nodesource/distributions#debinstall

RUN apt-get update -y 
RUN apt-get install curl -y

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
RUN apt-get install -y nodejs

# install Angular cli
RUN npm install -g @angular/cli@15.2.10
