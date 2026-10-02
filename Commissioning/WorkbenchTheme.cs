using System.Windows;
using System.Windows.Controls;
using System.Windows.Markup;
using System.Windows.Media;

namespace TwoLinkCommissioning;

// Light "engineering software" look (design 3a): flat 2 px controls, gray panels, hairline borders, blue accents.
internal static class WorkbenchTheme
{
    public static readonly Brush Ink = Brush("#1F2933"), Sub = Brush("#3E4C59"), Muted = Brush("#616E7C"), Blue = Brush("#1F5FBF"),
        Teal = Brush("#087F8C"), Red = Brush("#C62828"), Line = Brush("#C9D1DB"), Background = Brush("#ECEFF3"),
        Bar = Brush("#F7F9FB"), Head = Brush("#E3E8EE"), Group = Brush("#F0F3F7"), Field = Brush("#B8C2CF"),
        Faint = Brush("#EEF1F5"), GridLine = Brush("#E3E8EE"), Idle = Brush("#9AA5B1"), Ok = Brush("#2E7D32"), Warn = Brush("#E0A100");
    public static readonly FontFamily Mono = new("Consolas");
    public static SolidColorBrush Brush(string color) => (SolidColorBrush)new BrushConverter().ConvertFromString(color)!;
    public static void Apply(FrameworkElement root)
    {
        root.Resources.MergedDictionaries.Add((ResourceDictionary)XamlReader.Parse("""
        <ResourceDictionary xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation" xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml">
          <Style TargetType="Button">
            <Setter Property="Background" Value="#F7F9FB"/><Setter Property="Foreground" Value="#1F2933"/>
            <Setter Property="BorderBrush" Value="#C9D1DB"/><Setter Property="BorderThickness" Value="1"/>
            <Setter Property="Padding" Value="12,3"/><Setter Property="Margin" Value="2"/>
            <Setter Property="MinHeight" Value="26"/><Setter Property="FontSize" Value="12"/>
            <Setter Property="Cursor" Value="Hand"/><Setter Property="HorizontalContentAlignment" Value="Center"/>
            <Setter Property="Template"><Setter.Value><ControlTemplate TargetType="Button">
              <Border x:Name="surface" CornerRadius="2" Background="{TemplateBinding Background}" BorderBrush="{TemplateBinding BorderBrush}" BorderThickness="{TemplateBinding BorderThickness}" Padding="{TemplateBinding Padding}">
                <ContentPresenter x:Name="caption" HorizontalAlignment="{TemplateBinding HorizontalContentAlignment}" VerticalAlignment="Center" RecognizesAccessKey="True" TextElement.Foreground="{TemplateBinding Foreground}"/>
              </Border>
              <ControlTemplate.Triggers>
                <Trigger Property="IsMouseOver" Value="True"><Setter TargetName="surface" Property="Opacity" Value="0.85"/></Trigger>
                <Trigger Property="IsPressed" Value="True"><Setter TargetName="surface" Property="Opacity" Value="0.65"/></Trigger>
                <Trigger Property="IsKeyboardFocused" Value="True"><Setter TargetName="surface" Property="BorderBrush" Value="#1F2933"/><Setter TargetName="surface" Property="BorderThickness" Value="2"/></Trigger>
                <Trigger Property="IsEnabled" Value="False"><Setter TargetName="surface" Property="Opacity" Value="0.5"/><Setter TargetName="surface" Property="Background" Value="#ECEFF3"/><Setter TargetName="surface" Property="BorderBrush" Value="#C9D1DB"/><Setter TargetName="caption" Property="TextElement.Foreground" Value="#677386"/><Setter Property="Cursor" Value="Arrow"/></Trigger>
              </ControlTemplate.Triggers>
            </ControlTemplate></Setter.Value></Setter>
          </Style>
          <Style TargetType="TextBox"><Setter Property="FontSize" Value="12"/><Setter Property="Foreground" Value="#1F2933"/><Setter Property="BorderBrush" Value="#B8C2CF"/><Setter Property="Padding" Value="6,3"/><Setter Property="VerticalContentAlignment" Value="Center"/><Setter Property="SelectionBrush" Value="#B2CBFF"/></Style>
          <Style TargetType="ComboBox"><Setter Property="FontSize" Value="12"/><Setter Property="MinHeight" Value="24"/><Setter Property="Padding" Value="5,2"/><Setter Property="VerticalContentAlignment" Value="Center"/></Style>
          <!-- The default determinate bar also animates its glow. Keep rendering finite so input and heartbeats can run. -->
          <Style TargetType="ProgressBar">
            <Setter Property="Template"><Setter.Value><ControlTemplate TargetType="ProgressBar">
              <Grid>
                <Border x:Name="PART_Track" Background="{TemplateBinding Background}"/>
                <Border x:Name="PART_Indicator" HorizontalAlignment="Left" Background="{TemplateBinding Foreground}"/>
              </Grid>
            </ControlTemplate></Setter.Value></Setter>
          </Style>
          <Style TargetType="TabItem">
            <Setter Property="Padding" Value="14,4"/><Setter Property="FontSize" Value="12"/><Setter Property="Foreground" Value="#1F2933"/>
            <Setter Property="Background" Value="Transparent"/>
            <Setter Property="Template"><Setter.Value><ControlTemplate TargetType="TabItem">
              <Border x:Name="tab" Background="{TemplateBinding Background}" Padding="{TemplateBinding Padding}" BorderThickness="1,1,1,0" BorderBrush="Transparent">
                <ContentPresenter x:Name="tabTitle" ContentSource="Header" VerticalAlignment="Center" TextElement.Foreground="#616E7C"/>
              </Border>
              <ControlTemplate.Triggers>
                <Trigger Property="IsSelected" Value="True"><Setter TargetName="tabTitle" Property="TextElement.Foreground" Value="#1F5FBF"/><Setter TargetName="tabTitle" Property="TextElement.FontWeight" Value="Bold"/><Setter Property="Background" Value="White"/><Setter TargetName="tab" Property="BorderBrush" Value="#C9D1DB"/></Trigger>
                <Trigger Property="IsKeyboardFocused" Value="True"><Setter TargetName="tab" Property="BorderBrush" Value="#1F2933"/></Trigger>
              </ControlTemplate.Triggers>
            </ControlTemplate></Setter.Value></Setter>
          </Style>
          <Style TargetType="TabControl">
            <Setter Property="Background" Value="White"/><Setter Property="BorderBrush" Value="#C9D1DB"/><Setter Property="Padding" Value="0"/>
            <Setter Property="Template"><Setter.Value><ControlTemplate TargetType="TabControl">
              <Grid>
                <Grid.RowDefinitions><RowDefinition Height="Auto"/><RowDefinition/></Grid.RowDefinitions>
                <Border Background="#E3E8EE" BorderBrush="#C9D1DB" BorderThickness="0,0,0,1"><TabPanel IsItemsHost="True" Margin="8,3,8,0"/></Border>
                <Border Grid.Row="1" Background="{TemplateBinding Background}" Padding="{TemplateBinding Padding}"><ContentPresenter ContentSource="SelectedContent"/></Border>
              </Grid>
            </ControlTemplate></Setter.Value></Setter>
          </Style>
        </ResourceDictionary>
        """));
    }
    public static TextBlock Text(string text, double size = 12, Brush? color = null) => new() {
        Text = text, FontSize = size, Foreground = color ?? Ink, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(3,4,3,4)
    };
    public static void Primary(Button b) { b.Background = Blue; b.Foreground = Brushes.White; b.BorderBrush = Blue; }
    public static Border Edge(UIElement child, Brush? fill = null, Thickness? edge = null) => new() {
        Background = fill ?? Bar, BorderBrush = Line, BorderThickness = edge ?? new Thickness(0,0,0,1), Child = child
    };

    // Property-grid pieces: a dock caption, a group header, and a "name | value | unit" row.
    public static Border Caption(string key) => Edge(Lang.Bind(new TextBlock { FontWeight = FontWeights.Bold, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(10,0,10,0) }, TextBlock.TextProperty, key), Head);
    // A titled group whose header folds the body away (▾ / ▸); `trailing` sits at the right end of the header.
    private static readonly ControlTemplate SectionTemplate = (ControlTemplate)XamlReader.Parse("""
        <ControlTemplate xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation" xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml" TargetType="Expander">
          <DockPanel>
            <ToggleButton DockPanel.Dock="Top" Content="{TemplateBinding Header}" IsChecked="{Binding IsExpanded, Mode=TwoWay, RelativeSource={RelativeSource TemplatedParent}}" Cursor="Hand">
              <ToggleButton.Template><ControlTemplate TargetType="ToggleButton">
                <Border x:Name="bar" Background="#F0F3F7" BorderBrush="#DDE3EA" BorderThickness="0,0,0,1" Padding="6,3,6,3">
                  <DockPanel>
                    <TextBlock x:Name="glyph" DockPanel.Dock="Left" Text="▾" Width="16" Foreground="#616E7C" VerticalAlignment="Center"/>
                    <ContentPresenter VerticalAlignment="Center"/>
                  </DockPanel>
                </Border>
                <ControlTemplate.Triggers>
                  <Trigger Property="IsChecked" Value="False"><Setter TargetName="glyph" Property="Text" Value="▸"/></Trigger>
                  <Trigger Property="IsMouseOver" Value="True"><Setter TargetName="bar" Property="Background" Value="#E8EDF3"/></Trigger>
                  <Trigger Property="IsKeyboardFocused" Value="True"><Setter TargetName="bar" Property="BorderBrush" Value="#1F5FBF"/></Trigger>
                </ControlTemplate.Triggers>
              </ControlTemplate></ToggleButton.Template>
            </ToggleButton>
            <ContentPresenter x:Name="body" Visibility="Collapsed"/>
          </DockPanel>
          <ControlTemplate.Triggers><Trigger Property="IsExpanded" Value="True"><Setter TargetName="body" Property="Visibility" Value="Visible"/></Trigger></ControlTemplate.Triggers>
        </ControlTemplate>
        """);
    public static Expander Section(string title, UIElement content, UIElement? trailing = null, bool expanded = true)
    {
        var header = new DockPanel();
        if (trailing is not null) { DockPanel.SetDock(trailing, Dock.Right); header.Children.Add(trailing); }
        header.Children.Add(Lang.Bind(new TextBlock { FontWeight = FontWeights.Bold, Foreground = Sub, VerticalAlignment = VerticalAlignment.Center }, TextBlock.TextProperty, title));
        return new() { Template = SectionTemplate, Header = header, Content = content, IsExpanded = expanded };
    }
    public static Border Row(UIElement label, Control value, string unit = "")
    {
        value.Width = double.NaN; value.Margin = new Thickness(0); value.MinHeight = 22; value.VerticalAlignment = VerticalAlignment.Center;
        if (value is TextBox box) { box.HorizontalContentAlignment = HorizontalAlignment.Right; box.Padding = new Thickness(6,1,6,1); box.FontFamily = Mono; }
        var grid = new Grid { Margin = new Thickness(10,2,10,2) };
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(104) });
        grid.ColumnDefinitions.Add(new ColumnDefinition());
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(40) });
        if (label is FrameworkElement l) { l.VerticalAlignment = VerticalAlignment.Center; l.Margin = new Thickness(0,0,6,0); }
        Grid.SetColumn(value, 1);
        var u = new TextBlock { Text = unit, Foreground = Muted, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(6,0,0,0) }; Grid.SetColumn(u, 2);
        grid.Children.Add(label); grid.Children.Add(value); grid.Children.Add(u);
        return new() { Background = Brushes.White, BorderBrush = Faint, BorderThickness = new Thickness(0,0,0,1), Child = grid };
    }
    // A value row with a full-width slider under it: the box shows the exact value and still takes typed input.
    public static Border SliderRow(UIElement label, Slider slider, TextBox box, string unit = "")
    {
        var top = Row(label, box, unit); top.BorderThickness = new Thickness(0);
        slider.Margin = new Thickness(10,0,10,4); slider.IsMoveToPointEnabled = true;
        return new() { Background = Brushes.White, BorderBrush = Faint, BorderThickness = new Thickness(0,0,0,1), Child = new StackPanel { Children = { top, slider } } };
    }
    public static TextBlock Label(string text, Brush? color = null) => new() { Text = text, Foreground = color ?? Sub, TextTrimming = TextTrimming.CharacterEllipsis };
    public static TextBlock LocalLabel(string key) => Lang.Bind(Lang.Bind(Label(Lang.T(key)), TextBlock.TextProperty, key), FrameworkElement.ToolTipProperty, key);
}
