using System;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Reactive;
using System.Reactive.Concurrency;
using System.Reactive.Disposables;
using System.Reactive.Linq;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Bonsai.Harp;
using Harp.SyringePump.Design.Views;
using MsBox.Avalonia;
using MsBox.Avalonia.Enums;
using ReactiveUI;
using ReactiveUI.Fody.Helpers;

namespace Harp.SyringePump.Design.ViewModels;


public class SyringePumpViewModel : ViewModelBase
{
    public string AppVersion { get; set; }
    public ReactiveCommand<Unit, Unit> LoadDeviceInformation { get; }

    public ReactiveCommand<Unit, Unit> StartProtocolCommand { get; }

    #region Connection Information

    [Reactive] public ObservableCollection<string> Ports { get; set; }
    [Reactive] public string SelectedPort { get; set; }
    [Reactive] public bool Connected { get; set; }
    [Reactive] public string ConnectButtonText { get; set; } = "Connect";
    public ReactiveCommand<Unit, Unit> ConnectAndGetBaseInfoCommand { get; }

    #endregion

    #region Operations

    public ReactiveCommand<bool, Unit> SaveConfigurationCommand { get; }
    public ReactiveCommand<Unit, Unit> ResetConfigurationCommand { get; }

    #endregion

    #region Device basic information

    [Reactive] public int DeviceID { get; set; }
    [Reactive] public string DeviceName { get; set; }
    [Reactive] public HarpVersion HardwareVersion { get; set; }
    [Reactive] public HarpVersion FirmwareVersion { get; set; }
    [Reactive] public int SerialNumber { get; set; }

    #endregion

    #region Registers

    [Reactive] public EnableFlag EnableMotorDriver { get; set; }
    [Reactive] public EnableFlag EnableProtocol { get; set; }
    [Reactive] public StepState Step { get; set; }
    [Reactive] public DirectionState Direction { get; set; }
    [Reactive] public ForwardSwitchState ForwardSwitch { get; set; }
    [Reactive] public ReverseSwitchState ReverseSwitch { get; set; }
    [Reactive] public DigitalInputs DigitalInputState { get; set; }
    [Reactive] public DigitalOutputs DigitalOutputSet { get; set; }
    [Reactive] public DigitalOutputs DigitalOutputClear { get; set; }
    [Reactive] public DO0SyncConfig DO0Sync { get; set; }
    [Reactive] public DO1SyncConfig DO1Sync { get; set; }
    [Reactive] public DI0TriggerConfig DI0Trigger { get; set; }
    [Reactive] public StepModeType StepMode { get; set; }
    [Reactive] public ushort ProtocolStepCount { get; set; }
    [Reactive] public ushort ProtocolPeriod { get; set; }
    [Reactive] public PumpEvents EnableEvents { get; set; }
    [Reactive] public ProtocolState Protocol { get; set; }
    [Reactive] public ProtocolDirectionState ProtocolDirection { get; set; }

    #endregion

    #region Array Collections


    #endregion

    #region Events Flags

    public bool IsStepEnabled
    {
        get
        {
            return EnableEvents.HasFlag(PumpEvents.Step);
        }
        set
        {
            if (value)
            {
                EnableEvents |= PumpEvents.Step;
            }
            else
            {
                EnableEvents &= ~PumpEvents.Step;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsStepEnabled));
            this.RaisePropertyChanged(nameof(EnableEvents));
        }
    }

    public bool IsDirectionEnabled
    {
        get
        {
            return EnableEvents.HasFlag(PumpEvents.Direction);
        }
        set
        {
            if (value)
            {
                EnableEvents |= PumpEvents.Direction;
            }
            else
            {
                EnableEvents &= ~PumpEvents.Direction;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsDirectionEnabled));
            this.RaisePropertyChanged(nameof(EnableEvents));
        }
    }

    public bool IsForwardSwitchEnabled
    {
        get
        {
            return EnableEvents.HasFlag(PumpEvents.ForwardSwitch);
        }
        set
        {
            if (value)
            {
                EnableEvents |= PumpEvents.ForwardSwitch;
            }
            else
            {
                EnableEvents &= ~PumpEvents.ForwardSwitch;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsForwardSwitchEnabled));
            this.RaisePropertyChanged(nameof(EnableEvents));
        }
    }

    public bool IsReverseSwitchEnabled
    {
        get
        {
            return EnableEvents.HasFlag(PumpEvents.ReverseSwitch);
        }
        set
        {
            if (value)
            {
                EnableEvents |= PumpEvents.ReverseSwitch;
            }
            else
            {
                EnableEvents &= ~PumpEvents.ReverseSwitch;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsReverseSwitchEnabled));
            this.RaisePropertyChanged(nameof(EnableEvents));
        }
    }

    public bool IsDigitalInputEnabled
    {
        get
        {
            return EnableEvents.HasFlag(PumpEvents.DigitalInput);
        }
        set
        {
            if (value)
            {
                EnableEvents |= PumpEvents.DigitalInput;
            }
            else
            {
                EnableEvents &= ~PumpEvents.DigitalInput;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsDigitalInputEnabled));
            this.RaisePropertyChanged(nameof(EnableEvents));
        }
    }

    public bool IsProtocolEnabled
    {
        get
        {
            return EnableEvents.HasFlag(PumpEvents.Protocol);
        }
        set
        {
            if (value)
            {
                EnableEvents |= PumpEvents.Protocol;
            }
            else
            {
                EnableEvents &= ~PumpEvents.Protocol;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsProtocolEnabled));
            this.RaisePropertyChanged(nameof(EnableEvents));
        }
    }

    #endregion

    #region DigitalInputs_DigitalInputState Flags

    public bool IsDI0Enabled_DigitalInputState
    {
        get
        {
            return DigitalInputState.HasFlag(DigitalInputs.DI0);
        }
        set
        {
            if (value)
            {
                DigitalInputState |= DigitalInputs.DI0;
            }
            else
            {
                DigitalInputState &= ~DigitalInputs.DI0;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsDI0Enabled_DigitalInputState));
            this.RaisePropertyChanged(nameof(DigitalInputState));
        }
    }

    #endregion

    #region DigitalOutputs_DigitalOutputSet Flags

    public bool IsDO0Enabled_DigitalOutputSet
    {
        get
        {
            return DigitalOutputSet.HasFlag(DigitalOutputs.DO0);
        }
        set
        {
            if (value)
            {
                DigitalOutputSet |= DigitalOutputs.DO0;
            }
            else
            {
                DigitalOutputSet &= ~DigitalOutputs.DO0;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsDO0Enabled_DigitalOutputSet));
            this.RaisePropertyChanged(nameof(DigitalOutputSet));
        }
    }

    public bool IsDO1Enabled_DigitalOutputSet
    {
        get
        {
            return DigitalOutputSet.HasFlag(DigitalOutputs.DO1);
        }
        set
        {
            if (value)
            {
                DigitalOutputSet |= DigitalOutputs.DO1;
            }
            else
            {
                DigitalOutputSet &= ~DigitalOutputs.DO1;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsDO1Enabled_DigitalOutputSet));
            this.RaisePropertyChanged(nameof(DigitalOutputSet));
        }
    }

    #endregion

    #region DigitalOutputs_DigitalOutputClear Flags

    public bool IsDO0Enabled_DigitalOutputClear
    {
        get
        {
            return DigitalOutputClear.HasFlag(DigitalOutputs.DO0);
        }
        set
        {
            if (value)
            {
                DigitalOutputClear |= DigitalOutputs.DO0;
            }
            else
            {
                DigitalOutputClear &= ~DigitalOutputs.DO0;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsDO0Enabled_DigitalOutputClear));
            this.RaisePropertyChanged(nameof(DigitalOutputClear));
        }
    }

    public bool IsDO1Enabled_DigitalOutputClear
    {
        get
        {
            return DigitalOutputClear.HasFlag(DigitalOutputs.DO1);
        }
        set
        {
            if (value)
            {
                DigitalOutputClear |= DigitalOutputs.DO1;
            }
            else
            {
                DigitalOutputClear &= ~DigitalOutputs.DO1;
            }

            // Notify the UI about the change
            this.RaisePropertyChanged(nameof(IsDO1Enabled_DigitalOutputClear));
            this.RaisePropertyChanged(nameof(DigitalOutputClear));
        }
    }

    #endregion

    #region Application State

    [ObservableAsProperty] public bool IsLoadingPorts { get; }
    [ObservableAsProperty] public bool IsConnecting { get; }
    [ObservableAsProperty] public bool IsResetting { get; }
    [ObservableAsProperty] public bool IsSaving { get; }
    [ObservableAsProperty] public bool IsRunningProtocol { get; }

    [Reactive] public bool ShowWriteMessages { get; set; }
    [Reactive] public ObservableCollection<string> HarpEvents { get; set; } = new ObservableCollection<string>();
    [Reactive] public ObservableCollection<string> SentMessages { get; set; } = new ObservableCollection<string>();

    public ReactiveCommand<Unit, Unit> ShowAboutCommand { get; private set; }
    public ReactiveCommand<Unit, Unit> ClearMessagesCommand { get; private set; }
    public ReactiveCommand<Unit, Unit> ShowMessagesCommand { get; private set; }

    #endregion

    private Harp.SyringePump.AsyncDevice? _device;
    private IObservable<string> _deviceEventsObservable;
    private IDisposable? _deviceEventsSubscription;

    public SyringePumpViewModel()
    {
        var assembly = typeof(SyringePumpViewModel).Assembly;
        var informationVersion = assembly.GetName().Version;
        if (informationVersion != null)
            AppVersion = $"v{informationVersion.Major}.{informationVersion.Minor}.{informationVersion.Build}";

        Ports = new ObservableCollection<string>();

        ClearMessagesCommand = ReactiveCommand.Create(() => { SentMessages.Clear(); });
        ShowMessagesCommand = ReactiveCommand.Create(() => { ShowWriteMessages = !ShowWriteMessages; });

        LoadDeviceInformation = ReactiveCommand.CreateFromObservable(LoadUsbInformation);
        LoadDeviceInformation.IsExecuting.ToPropertyEx(this, x => x.IsLoadingPorts);
        LoadDeviceInformation.ThrownExceptions.Subscribe(ex =>
            Console.WriteLine($"Error loading device information with exception: {ex.Message}"));
        //Log.Error(ex, "Error loading device information with exception: {Exception}", ex));

        // can connect if there is a selection and also if the new selection is different than the old one
        var canConnect = this.WhenAnyValue(x => x.SelectedPort)
            .Select(selectedPort => !string.IsNullOrEmpty(selectedPort));

        ShowAboutCommand = ReactiveCommand.CreateFromTask(async () =>
                await new About() { DataContext = new AboutViewModel() }.ShowDialog(
                    (Application.Current?.ApplicationLifetime as IClassicDesktopStyleApplicationLifetime)?.MainWindow));

        ConnectAndGetBaseInfoCommand = ReactiveCommand.CreateFromTask(ConnectAndGetBaseInfo, canConnect);
        ConnectAndGetBaseInfoCommand.IsExecuting.ToPropertyEx(this, x => x.IsConnecting);
        ConnectAndGetBaseInfoCommand.ThrownExceptions.Subscribe(ex =>
            //Log.Error(ex, "Error connecting to device with error: {Exception}", ex));
            Console.WriteLine($"Error connecting to device with error: {ex}"));

        var canChangeConfig = this.WhenAnyValue(x => x.Connected).Select(connected => connected);
        // Handle Save and Reset
        SaveConfigurationCommand =
            ReactiveCommand.CreateFromObservable<bool, Unit>(SaveConfiguration, canChangeConfig);
        SaveConfigurationCommand.IsExecuting.ToPropertyEx(this, x => x.IsSaving);
        SaveConfigurationCommand.ThrownExceptions.Subscribe(ex =>
            //Log.Error(ex, "Error saving configuration with error: {Exception}", ex));
            Console.WriteLine($"Error saving configuration with error: {ex}"));

        ResetConfigurationCommand = ReactiveCommand.CreateFromObservable(ResetConfiguration, canChangeConfig);
        ResetConfigurationCommand.IsExecuting.ToPropertyEx(this, x => x.IsResetting);
        ResetConfigurationCommand.ThrownExceptions.Subscribe(ex =>
            //Log.Error(ex, "Error resetting device configuration with error: {Exception}", ex));
            Console.WriteLine($"Error resetting device configuration with error: {ex}"));

        var canStartProtocol = this.WhenAnyValue(
            x => x.Connected,
            x => x.Protocol,
            (connected, protocol) => connected && protocol != ProtocolState.Running
        );
        
        StartProtocolCommand = ReactiveCommand.CreateFromObservable(StartProtocol, canStartProtocol);
        StartProtocolCommand.IsExecuting.ToPropertyEx(this, x => x.IsRunningProtocol);
        StartProtocolCommand.ThrownExceptions.Subscribe(ex =>
            //Log.Error(ex, "Error starting protocol with error: {Exception}", ex));
            Console.WriteLine($"Error starting protocol with error: {ex}"));

        this.WhenAnyValue(x => x.Connected)
            .Subscribe(x => { ConnectButtonText = x ? "Disconnect" : "Connect"; });

        this.WhenAnyValue(x => x.EnableEvents)
            .Subscribe(x =>
            {
                IsStepEnabled = x.HasFlag(PumpEvents.Step);
                IsDirectionEnabled = x.HasFlag(PumpEvents.Direction);
                IsForwardSwitchEnabled = x.HasFlag(PumpEvents.ForwardSwitch);
                IsReverseSwitchEnabled = x.HasFlag(PumpEvents.ReverseSwitch);
                IsDigitalInputEnabled = x.HasFlag(PumpEvents.DigitalInput);
                IsProtocolEnabled = x.HasFlag(PumpEvents.Protocol);
            });


        // handle the events from the device0
        // When Connected changes subscribe/unsubscribe the device events.
        this.WhenAnyValue(x => x.Connected)
            .ObserveOn(RxApp.MainThreadScheduler)
            .Subscribe(isConnected =>
            {
                if (isConnected && _deviceEventsObservable != null)
                {
                    // Subscribe on the UI thread so that the HarpEvents collection can be updated safely.
                    _deviceEventsSubscription = _deviceEventsObservable
                        .ObserveOn(RxApp.MainThreadScheduler)
                        .Subscribe(
                            msg => HarpEvents.Add(msg.ToString()),
                            ex => Debug.WriteLine($"Error in device events: {ex}")
                        );
                }
                else
                {
                    // Dispose subscription and clear messages.
                    _deviceEventsSubscription?.Dispose();
                    _deviceEventsSubscription = null;
                }
            });

        this.WhenAnyValue(x => x.DigitalInputState)
            .Subscribe(x =>
            {
                IsDI0Enabled_DigitalInputState = x.HasFlag(DigitalInputs.DI0);
            });

        this.WhenAnyValue(x => x.DigitalOutputSet)
            .Subscribe(x =>
            {
                IsDO0Enabled_DigitalOutputSet = x.HasFlag(DigitalOutputs.DO0);
                IsDO1Enabled_DigitalOutputSet = x.HasFlag(DigitalOutputs.DO1);
            });

        this.WhenAnyValue(x => x.DigitalOutputClear)
            .Subscribe(x =>
            {
                IsDO0Enabled_DigitalOutputClear = x.HasFlag(DigitalOutputs.DO0);
                IsDO1Enabled_DigitalOutputClear = x.HasFlag(DigitalOutputs.DO1);
            });
        
        // force initial population of currently connected ports
        LoadUsbInformation();
    }

    private IObservable<Unit> LoadUsbInformation()
    {
        return Observable.Start(() =>
        {
            var devices = SerialPort.GetPortNames();

            if (OperatingSystem.IsMacOS())
                // except with Bluetooth in the name
                Ports = new ObservableCollection<string>(devices.Where(d => d.Contains("cu.")).Except(devices.Where(d => d.Contains("Bluetooth"))));
            else
                Ports = new ObservableCollection<string>(devices);

            Console.WriteLine("Loaded USB information");
            //Log.Information("Loaded USB information");
        });
    }

    private async Task ConnectAndGetBaseInfo()
    {
        if (string.IsNullOrEmpty(SelectedPort))
            throw new Exception("invalid parameter");

        if (Connected)
        {
            _device?.Dispose();
            _device = null;
            Connected = false;
            SentMessages.Clear();
            return;
        }

        try
        {
            _device = await Harp.SyringePump.Device.CreateAsync(SelectedPort);
        }
        catch (OperationCanceledException ex)
        {
            Console.WriteLine($"Error connecting to device with error: {ex}");
            //Log.Error(ex, "Error connecting to device with error: {Exception}", ex);
            var messageBoxStandardWindow = MessageBoxManager
                .GetMessageBoxStandard("Unexpected device found",
                    "Timeout when trying to connect to a device. Most likely not an Harp device.",
                    icon: Icon.Error);
            await messageBoxStandardWindow.ShowAsync();
            _device?.Dispose();
            _device = null;
            return;

        }
        catch (HarpException ex)
        {
            Console.WriteLine($"Error connecting to device with error: {ex}");
            //Log.Error(ex, "Error connecting to device with error: {Exception}", ex);

            var messageBoxStandardWindow = MessageBoxManager
                .GetMessageBoxStandard("Unexpected device found",
                    ex.Message,
                    icon: Icon.Error);
            await messageBoxStandardWindow.ShowAsync();

            _device?.Dispose();
            _device = null;
            return;
        }
        catch (UnauthorizedAccessException ex)
        {
            Console.WriteLine($"COM port still in use and most likely not the expected Harp device");
            var messageBoxStandardWindow = MessageBoxManager
                .GetMessageBoxStandard("Unexpected device found",
                    $"COM port still in use and most likely not the expected Harp device.{Environment.NewLine}Specific error: {ex.Message}",
                    icon: Icon.Error);
            await messageBoxStandardWindow.ShowAsync();

            _device?.Dispose();
            _device = null;
            return;
        }

        // Clear the sent messages list
        SentMessages.Clear();

        //Log.Information("Attempting connection to port \'{SelectedPort}\'", SelectedPort);
        Console.WriteLine($"Attempting connection to port \'{SelectedPort}\'");

        DeviceID = await _device.ReadWhoAmIAsync();
        DeviceName = await _device.ReadDeviceNameAsync();
        HardwareVersion = await _device.ReadHardwareVersionAsync();
        FirmwareVersion = await _device.ReadFirmwareVersionAsync();
        try
        {
            // some devices may not have a serial number
            SerialNumber = await _device.ReadSerialNumberAsync();
        }
        catch (HarpException)
        {
            // Device does not have a serial number, simply continue by ignoring the exception
        }

        /*****************************************************************
        * TODO: Please REVIEW all these registers and update the values
        * ****************************************************************/
        EnableMotorDriver = await _device.ReadEnableMotorDriverAsync();
        EnableProtocol = await _device.ReadEnableProtocolAsync();
        Step = await _device.ReadStepAsync();
        Direction = await _device.ReadDirectionAsync();
        ForwardSwitch = await _device.ReadForwardSwitchAsync();
        ReverseSwitch = await _device.ReadReverseSwitchAsync();
        DigitalInputState = await _device.ReadDigitalInputStateAsync();
        DigitalOutputSet = await _device.ReadDigitalOutputSetAsync();
        DigitalOutputClear = await _device.ReadDigitalOutputClearAsync();
        DO0Sync = await _device.ReadDO0SyncAsync();
        DO1Sync = await _device.ReadDO1SyncAsync();
        DI0Trigger = await _device.ReadDI0TriggerAsync();
        StepMode = await _device.ReadStepModeAsync();
        ProtocolStepCount = await _device.ReadProtocolStepCountAsync();
        ProtocolPeriod = await _device.ReadProtocolPeriodAsync();
        EnableEvents = await _device.ReadEnableEventsAsync();
        Protocol = await _device.ReadProtocolAsync();
        ProtocolDirection = await _device.ReadProtocolDirectionAsync();


        // generate observable for the _deviceSync
        _deviceEventsObservable = GenerateEventMessages();

        Connected = true;

        //Log.Information("Connected to device");
        Console.WriteLine("Connected to device");
    }

    private IObservable<string> GenerateEventMessages()
    {
        return Observable.Create<string>(async (observer, cancellationToken) =>
        {
            // Loop until cancellation is requested or the device is no longer available.
            while (!cancellationToken.IsCancellationRequested && _device != null)
            {
                // Capture local reference and check for null.
                var device = _device;
                if (device == null)
                {
                    observer.OnCompleted();
                    break;
                }

                try
                {
                    // Check if Step event is enabled
                    if (IsStepEnabled)
                    {
                        var result = await device.ReadStepAsync(cancellationToken);
                        Step = result;
                        observer.OnNext($"Step: {result}");
                    }

                    // Check if Direction event is enabled
                    if (IsDirectionEnabled)
                    {
                        var result = await device.ReadDirectionAsync(cancellationToken);
                        Direction = result;
                        observer.OnNext($"Direction: {result}");
                    }

                    // Check if ForwardSwitch event is enabled
                    if (IsForwardSwitchEnabled)
                    {
                        var result = await device.ReadForwardSwitchAsync(cancellationToken);
                        ForwardSwitch = result;
                        observer.OnNext($"ForwardSwitch: {result}");
                    }

                    // Check if ReverseSwitch event is enabled
                    if (IsReverseSwitchEnabled)
                    {
                        var result = await device.ReadReverseSwitchAsync(cancellationToken);
                        ReverseSwitch = result;
                        observer.OnNext($"ReverseSwitch: {result}");
                    }

                    // Check if DigitalInput event is enabled
                    if (IsDigitalInputEnabled)
                    {
                        var result = await device.ReadDigitalInputStateAsync(cancellationToken);
                        DigitalInputState = result;
                        observer.OnNext($"DigitalInput: {result}");
                    }

                    // Check if Protocol event is enabled
                    if (IsProtocolEnabled)
                    {
                        var result = await device.ReadProtocolAsync(cancellationToken);
                        Protocol = result;
                        observer.OnNext($"Protocol: {result}");
                    }

                    // Wait a short while before polling again. Adjust delay as necessary.
                    await Task.Delay(TimeSpan.FromMilliseconds(10), cancellationToken);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    observer.OnError(ex);
                    break;
                }
            }
            observer.OnCompleted();
            return Disposable.Empty;
        });
    }

    private IObservable<Unit> StartProtocol()
    {
        return Observable.StartAsync(async () =>
        {
            if (_device == null)
                return;

            // Write EnableProtocol message to device without changing it's value
            await WriteAndLogAsync(
                    value => _device.WriteEnableProtocolAsync(value),
                    EnableFlag.Enable,
                    "EnableProtocol");
        });
    }

    private IObservable<Unit> SaveConfiguration(bool savePermanently)
    {
        return Observable.StartAsync(async () =>
        {
            if (_device == null)
                throw new Exception("You need to connect to the device first");

            /*****************************************************************
            * TODO: Please REVIEW all these registers and update the values
            * ****************************************************************/
            await WriteAndLogAsync(
                value => _device.WriteEnableMotorDriverAsync(value),
                EnableMotorDriver,
                "EnableMotorDriver");
            await WriteAndLogAsync(
                value => _device.WriteEnableProtocolAsync(value),
                EnableProtocol,
                "EnableProtocol");
            await WriteAndLogAsync(
                value => _device.WriteDigitalOutputSetAsync(value),
                DigitalOutputSet,
                "DigitalOutputSet");
            await WriteAndLogAsync(
                value => _device.WriteDigitalOutputClearAsync(value),
                DigitalOutputClear,
                "DigitalOutputClear");
            await WriteAndLogAsync(
                value => _device.WriteDO0SyncAsync(value),
                DO0Sync,
                "DO0Sync");
            await WriteAndLogAsync(
                value => _device.WriteDO1SyncAsync(value),
                DO1Sync,
                "DO1Sync");
            await WriteAndLogAsync(
                value => _device.WriteDI0TriggerAsync(value),
                DI0Trigger,
                "DI0Trigger");
            await WriteAndLogAsync(
                value => _device.WriteStepModeAsync(value),
                StepMode,
                "StepMode");
            await WriteAndLogAsync(
                value => _device.WriteProtocolStepCountAsync(value),
                ProtocolStepCount,
                "ProtocolStepCount");
            await WriteAndLogAsync(
                value => _device.WriteProtocolPeriodAsync(value),
                ProtocolPeriod,
                "ProtocolPeriod");
            await WriteAndLogAsync(
                value => _device.WriteEnableEventsAsync(value),
                EnableEvents,
                "EnableEvents");
            await WriteAndLogAsync(
                value => _device.WriteProtocolDirectionAsync(value),
                ProtocolDirection,
                "ProtocolDirection");

            // Save the configuration to the device permanently
            if (savePermanently)
            {
                await WriteAndLogAsync(
                    value => _device.WriteResetDeviceAsync(value),
                    ResetFlags.Save,
                    "SavePermanently");
            }
        });
    }

    private IObservable<Unit> ResetConfiguration()
    {
        return Observable.StartAsync(async () =>
        {
            if (_device != null)
            {
                await WriteAndLogAsync(
                    value => _device.WriteResetDeviceAsync(value),
                    ResetFlags.RestoreDefault,
                    "ResetDevice");
            }
        });
    }

    private async Task WriteAndLogAsync<T>(Func<T, Task> writeFunc, T value, string registerName)
    {
        if (_device == null)
            throw new Exception("Device is not connected");

        await writeFunc(value);

        // Log the message to the SentMessages collection on the UI thread
        RxApp.MainThreadScheduler.Schedule(() =>
        {
            SentMessages.Add($"{DateTime.Now:HH:mm:ss.fff} - Write {registerName}: {value}");
        });
    }
}
