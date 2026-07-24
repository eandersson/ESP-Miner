import { Component, OnInit, OnDestroy } from '@angular/core';
import { Subject } from 'rxjs';
import { takeUntil } from 'rxjs/operators';
import { LayoutService } from '../../layout/service/app.layout.service';
import { ThemeService } from '../../services/theme.service';

interface ThemeOption {
  name: string;
  primaryColor: string;
}

@Component({
  selector: 'app-theme-config',
  templateUrl: './theme-config.component.html',
  styleUrls: ['./design-component.scss'],
  standalone: false
})
export class ThemeConfigComponent implements OnInit {
  selectedScheme: string;
  currentColor: string = '';
  themes: ThemeOption[] = [
    { name: 'Lichen', primaryColor: '#B8C9A5' },
    { name: 'Deep Lichen', primaryColor: '#8EA27E' },
    { name: 'Turquoise', primaryColor: '#0E7C7B' },
    { name: 'Deep Turquoise', primaryColor: '#0A5A5A' },
    { name: 'Horizon Teal', primaryColor: '#1F8A8A' },
    { name: 'Birch', primaryColor: '#F4F1E8' }
  ];

  private destroy$ = new Subject<void>();

  constructor(
    public layoutService: LayoutService,
    private themeService: ThemeService
  ) {
    this.selectedScheme = this.layoutService.config().colorScheme;
  }

  ngOnInit() {
    this.themeService.getThemeSettings()
      .pipe(takeUntil(this.destroy$))
      .subscribe({
        next: (settings) => {
          if (settings) {
            if (settings.colorScheme) {
              this.selectedScheme = settings.colorScheme;
            }
            if (settings.primaryColor) {
              this.currentColor = settings.primaryColor.toUpperCase();
              document.documentElement.style.setProperty('--color-primary', this.currentColor);
            }
          }
        },
        error: (error) => console.error('Error loading theme settings:', error)
      });
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }

  changeColorScheme(scheme: string) {
    this.selectedScheme = scheme;
    const config = { ...this.layoutService.config() };
    config.colorScheme = scheme;
    this.layoutService.config.set(config);

    this.themeService.saveThemeSettings({ colorScheme: scheme, primaryColor: this.currentColor })
      .pipe(takeUntil(this.destroy$))
      .subscribe({
        error: (error) => console.error('Error saving theme settings:', error)
      });
  }

  changeTheme(theme: ThemeOption) {
    this.currentColor = theme.primaryColor;
    document.documentElement.style.setProperty('--color-primary', this.currentColor);

    this.themeService.saveThemeSettings({
      colorScheme: this.selectedScheme,
      primaryColor: this.currentColor
    }).pipe(takeUntil(this.destroy$))
      .subscribe({
        error: (error) => console.error('Error saving theme settings:', error)
      });
  }

  onCustomColorChange(event: any) {
    const color = event.target.value;
    this.currentColor = color.toUpperCase();
    document.documentElement.style.setProperty('--color-primary', this.currentColor);

    this.themeService.saveThemeSettings({
      colorScheme: this.selectedScheme,
      primaryColor: this.currentColor
    }).pipe(takeUntil(this.destroy$))
      .subscribe({
        error: (error) => console.error('Error saving theme settings:', error)
      });
  }
}
