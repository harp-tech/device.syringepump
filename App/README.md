# Harp.SyringePump.App

This repository folder contains the Syringe Pump GUI (henceforth GUI) application that allows to configure the Syringe Pump device, developed by the Scientific Hardware Platform at the Champalimaud Foundation.

The Syringe Pump device is a Harp device and has all the inherent functionality of Harp devices.

The GUI was developed using [.NET](https://dotnet.microsoft.com/), [Avalonia](https://avaloniaui.net/) with ReactiveUI and makes direct use of the automatically generated Harp.SyringePump library that uses [Bonsai.Harp](https://github.com/bonsai-rx/harp) as a base reference.

As with other Harp devices, the Syringe Pump can also be used in [Bonsai](bonsai-rx.org/) using the [Bonsai.Harp.CF](https://github.com/bonsai-rx/harp.cf) package.

## Installation

Go to the Releases page to download the latest version for your Operating System.

Currently there are x64 builds for Windows and Linux. Mac builds will be available in the future.

Portable builds are also available.

### Linux

Since the application accesses the serial port, your user needs to be on the `dialout` group or equivalent.

There might be other alternatives to this, but at least on Ubuntu and Fedora que command that you need to run to add your user to the `dialout` group is:

```sh
sudo usermod -a -G dialout <USERNAME>
```

## For developers

### Build Windows installer using NSIS manually

- Install NSIS 3 on your Windows machine
- Build and publish the application using the .NET 8 SDK command-line tools (on the App folder)
  ```
    dotnet publish Harp.SyringePump.App.sln -r win-x64 -f net8.0 --self-contained /p:Configuration=Release
  ```
- Run makesis to generate the installer
    ```
     makensis.exe /DVERSION_MAJOR=0 /DVERSION_MINOR=1 /DVERSION_BUILD=0 .\Harp.SyringePump.nsi
    ```
- The installer will be available at `.\bin\Release\net8.0\win-x64\SyringePump.vx.x.x-win-x64.exe`

### Build .app image for macOS

The project uses dotnet-bundle (https://github.com/egramtel/dotnet-bundle) to generate a .app image for macOS.

To build the .app image, run the following commands on the solution folder:

```sh
dotnet restore -r osx-x64 -p:TargetFramework=net8.0
dotnet msbuild -t:BundleApp -p:RuntimeIdentifier=osx-x64 -property:Configuration=Release -p:UseAppHost=true -p:TargetFramework=net8.0
# or use -p:RuntimeIdentifier=osx-arm64 for Apple Silicon
```

### Build tar.gz package for Linux (either in Linux or WSL)

To build the tar.gz package, run the following commands on the solution folder:

First you need to make sure that the [`dotnet-packaging`](https://github.com/quamotion/dotnet-packaging) tool is installed

Run the following commands to install the tool:

```sh
dotnet tool install --global dotnet-tarball
```

Then run the following commands to build the tar.gz package:

```sh
dotnet restore -r linux-x64 -p:TargetFramework=net8.0
dotnet msbuild -p:RuntimeIdentifier=linux-x64 -property:Configuration=Release -p:UseAppHost=true -p:TargetFramework=net8.0 /t:CreateTarball
```

## Roadmap

See the [open issues](https://github.com/harp-tech/device.syringepump/issues) for a list of proposed features (and known issues).

## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Authors

Hardware & Software Platform of the Champalimaud Foundation.

### Main contributors

- Artur Silva
- Luís Teixeira

## License

See `LICENSE` file for more information.

